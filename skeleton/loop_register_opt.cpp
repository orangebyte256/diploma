#include "loop_register_opt.hpp"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"

#include <llvm/IR/Instructions.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/Analysis/LoopInfo.h>

#define VECTOR_SIZE 4
#define INTEGER_SIZE 32
#define VECTOR_POSTFIX "vector"
#define SCALAR_POSTFIX "scalar"
#define CAST_POSTFIX "cast"

using namespace boost;
namespace RegisterOpt{

using InstructionSet = std::set<Instruction*>;

template <typename T>
Instruction *insertElementInst(T instr, LLVMContext *myGlobalContext, Instruction *scalarInstr, llvm::Type *vectorType){
  return InsertElementInst::Create(UndefValue::get(vectorType), scalarInstr, ConstantInt::get(IntegerType::get(*myGlobalContext, INTEGER_SIZE),0), DFG::getInstructionName(scalarInstr) + VECTOR_POSTFIX, instr); 
}

Instruction* LoopRegisterOpt::makeCopyVectorInstruction(Instruction *scalarInstr,
 llvm::Type *vectorType){
  static LLVMContext myGlobalContext;
  if(!copyScalarInstruction[scalarInstr]){
    BasicBlock::iterator it(scalarInstr);
    it++;
    if(dyn_cast<PHINode>(scalarInstr))
      it = BasicBlock::iterator(it->getParent()->getFirstNonPHI());
    if(it == scalarInstr->getParent()->end()){
      copyScalarInstruction[scalarInstr] = insertElementInst(scalarInstr->getParent(), 
          &myGlobalContext, scalarInstr, vectorType);
    } else {
      copyScalarInstruction[scalarInstr] = insertElementInst(&(*it), 
          &myGlobalContext, scalarInstr, vectorType);
    }
  }
  return copyScalarInstruction[scalarInstr];
}

Instruction *extractElementInst(Instruction *instr, LLVMContext *myGlobalContext, Instruction *insert){
  return ExtractElementInst::Create(instr, ConstantInt::get(IntegerType::get(*myGlobalContext, 32),0), DFG::getInstructionName(instr) + SCALAR_POSTFIX, insert);
}

std::tuple<Instruction*,Instruction*> LoopRegisterOpt::makeCopyScalarInstruction(
Instruction *vectorInstr, Instruction *target){
  static LLVMContext myGlobalContext;
  auto key = std::make_tuple(vectorInstr, target->getParent());
  if(!copyVectorInstruction[key]){
    BasicBlock *newBB = supportBlocks[target->getParent()];
    for(auto &phi : *newBB){
      if(phi.getOperand(0) == vectorInstr){
        phi.mutateType(vectorInstr->getType());
        BasicBlock::iterator it(newBB->getFirstNonPHI());
        copyVectorInstruction[key] = extractElementInst(&phi, &myGlobalContext, &(*it));
        return std::make_tuple<Instruction*,Instruction*>((Instruction*)(&phi), (Instruction*)(copyVectorInstruction[key]));
      }   
    }
    BasicBlock::iterator it(newBB->getFirstNonPHI());
    if(target->getParent() == vectorInstr->getParent()){
      it = BasicBlock::iterator(vectorInstr);
      it++;
    }
    copyVectorInstruction[key] = extractElementInst(vectorInstr, &myGlobalContext, &(*it));
    return std::make_tuple<Instruction*,Instruction*>((Instruction*)(vectorInstr), (Instruction*)(copyVectorInstruction[key]));
  }
}

Value *LoopRegisterOpt::makeConstVectorCopy(Value *value){
  if(!vectorConstant[value])
    vectorConstant[value] = ConstantVector::getSplat(VECTOR_SIZE, (Constant*)(value));
  return vectorConstant[value];
}

Value* getConstantVector(Instruction *vectorInstr){
  std::vector<Constant *> myVector(4);
  myVector[0] = ConstantInt::get(IntegerType::get((vectorInstr)->getContext(), 1),1);
  for(int i = 1; i < VECTOR_SIZE; i++)
    myVector[i] = ConstantInt::get(IntegerType::get((vectorInstr)->getContext(), 1),0);
  return ConstantVector::get(ArrayRef<Constant *> (myVector));
}

void LoopRegisterOpt::optStore(Instruction *vectorInstr){
  static LLVMContext myGlobalContext;
  std::vector<llvm::Type *> arg_type;
  std::vector<Value *> args;
  Value *align = ConstantInt::get(IntegerType::get(vectorInstr->getContext(), 32), 1);
  auto constantVector = getConstantVector(vectorInstr);
  Value *ptr = (vectorInstr->getOperand(1));
  PointerType *type = PointerType::get(VectorType::get(ptr->getType()->getContainedType(0),4), 0); 
  Instruction *cast = new BitCastInst(ptr, type, DFG::getInstructionName((Instruction*)vectorInstr->getOperand(1)) + "cast", vectorInstr); 
 ArrayRef<llvm::Type *> arg_type_ar(arg_type);
  args.push_back((Value*)vectorInstr->getOperand(0));
  args.push_back((Value*)cast);
  args.push_back((Value*)align);
  args.push_back((Value*)(constantVector));
  for(auto arg : args)
    arg_type.push_back((llvm::Type*)arg->getType()); 
  Function *fun = Intrinsic::getDeclaration(vectorInstr->getModule(), Intrinsic::masked_store, arg_type);
  IRBuilder<> Builder(vectorInstr);
  Builder.CreateCall(fun, args);
  vectorInstr->eraseFromParent(); 
}

bool LoopRegisterOpt::isOptLoad(Instruction *scalarInstr){
  bool allVector = true;
  for(auto val : scalarInstr->users()){
    if(!val->getType()->isVectorTy())
      allVector = false;
  }
  if(!allVector)
    return false;  
  if(copyScalarInstruction[scalarInstr])
    return true;
  Instruction *ptr = dyn_cast<Instruction>(scalarInstr->getOperand(0));
  PointerType *type = PointerType::get(VectorType::get(ptr->getType()->getContainedType(0),VECTOR_SIZE), 0); 
  copyScalarInstruction[scalarInstr] = scalarInstr;
  scalarInstr->replaceUsesOfWith(scalarInstr->getOperand(0), new BitCastInst(ptr, type, DFG::getInstructionName(scalarInstr) + CAST_POSTFIX, scalarInstr));
  scalarInstr->mutateType(VectorType::get(ptr->getType()->getContainedType(0),VECTOR_SIZE));
}

InstructionSet replaceOperators(const InstructionSet &subgraph){
  InstructionSet result;
  for(auto instr : subgraph){
    if(dyn_cast<BinaryOperator>(instr) && instr->getOpcode() == Instruction::BinaryOps::Mul){
      auto val = BinaryOperator::Create(Instruction::SVMul, instr->getOperand(0), instr->getOperand(1), DFG::getInstructionName(instr), instr); 
      std::list<User*> usersList(instr->users().begin(), instr->users().end());
      for(auto tmp : usersList){
        tmp->replaceUsesOfWith(instr, val);
      }
      instr->eraseFromParent();
      result.insert(val);
    }
    else {
      result.insert(instr);
    }
  }
  return result; 
}


void LoopRegisterOpt::makeBlocks(BasicBlock *bb, BasicBlock *instr){
  if(!supportBlocks[instr]){
    supportBlocks[instr] = SplitEdge(bb, instr, dominatorTree,  loopInfo);
  }
}


void LoopRegisterOpt::makeAllBlocks(const InstructionSet& subgraph){
  for(auto instr : subgraph){
    for(auto val : instr->users()){
      if(dyn_cast<Instruction>(val))
        makeBlocks(instr->getParent(), ((Instruction*)(val))->getParent());
    } 
  }
}

std::map<Instruction*, InstructionSet> getAllUsers(const InstructionSet &subgraph){
  std::map<Instruction*,InstructionSet> res;
  for(auto instr : subgraph){
    res[instr] = InstructionSet();
    for(auto val : instr->users()){
      if(dyn_cast<Instruction>(val))
        res[instr].insert((Instruction*)val);
    }
  }
  return res;
}

void vectorize(const InstructionSet &subgraph){
  for(auto instr : subgraph){
    instr->mutateType(VectorType::get(instr->getType(), VECTOR_SIZE));
  }
}

void LoopRegisterOpt::addCopyInstruction(InstructionSet input){
  InstructionSet subgraph = replaceOperators(input);
  auto usersInstr = getAllUsers(subgraph);
  makeAllBlocks(subgraph);
  vectorize(subgraph);
  for(auto instr : subgraph){
    for(auto val = instr->op_begin(); val != instr->op_end(); val++){
      if(auto valConst = dyn_cast<Constant>(val)){ 
        instr->replaceUsesOfWith(valConst, makeConstVectorCopy(valConst));
        continue;
      }
      if(auto valInstr = dyn_cast<Instruction>(val)){
        if(subgraph.find(valInstr) != subgraph.end())
          continue;
        if(dyn_cast<LoadInst>(valInstr))
          if(isOptLoad(valInstr))
            continue;
        instr->replaceUsesOfWith(valInstr, makeCopyVectorInstruction(valInstr, instr->getType())); 
      }
    }
    for(auto val : usersInstr[instr]){
      if(subgraph.find(dyn_cast<Instruction>(val)) == subgraph.end()){
        if(dyn_cast<StoreInst>(dyn_cast<Instruction>(val))){
          optStore((StoreInst*)(val));
          continue;
        }
        std::tuple<Instruction*, Instruction*> res = makeCopyScalarInstruction(instr, dyn_cast<Instruction>(val));
        val->replaceUsesOfWith(get<0>(res), get<1>(res));
      }
    }
  }
}

void LoopRegisterOpt::writeGraph(std::string file){
  DFG::makeFullDFG(loop).writeGraph(file);
}


void LoopRegisterOpt::makeOpt(std::set<Instruction *> inst){
  InstructionSet instructionSet = innerDFG.getSubDFG(inst);
  for(auto t : instructionSet){
    t->dump();
  }
  addCopyInstruction(instructionSet);
}

LoopRegisterOpt::LoopRegisterOpt(Loop *loop, LoopInfo *loopInfo, DominatorTree *dominatorTree) : loop(loop), loopInfo(loopInfo), dominatorTree(dominatorTree), innerDFG(DFG::makeInternalDFG(loop)) {  
}

}

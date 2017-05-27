#ifndef LOOP_REGISTER_OPT_HPP
#define LOOP_REGISTER_OPT_HPP

#include "llvm/ADT/Statistic.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instruction.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/MapRegIRVreg.hpp"
#include "iostream"
#include <string>
#include <map>
#include "llvm/Pass.h"
#include "llvm/IR/InstIterator.h"
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/LoopInfo.h>


#include <iostream>
#include <vector>
#include <utility>
#include <tuple>
#include "dfg.hpp"


using namespace llvm;

namespace RegisterOpt{

class LoopRegisterOpt{
public:
  LoopRegisterOpt(Loop *loop_, LoopInfo *loopInfo, DominatorTree *dominatorTree);
//write data dependence graph to file for current loop
  void writeGraph(std::string file);
//execution of optimization
  void makeOpt(std::set<Instruction *>);
private:
  void makeBlocks(BasicBlock *bb, BasicBlock *instr);
//in some cases we need create new Basic Block for correct type swapping
  void makeAllBlocks(const std::set<Instruction *>&);
// transform scalar const value to vector
  Value *makeConstVectorCopy(Value *value);
// i.e scalar a = [some operation with vector b] -> b_scalar = cast to scalar b; a = [some operation with b_scalar]
  std::tuple<Instruction*, Instruction*> makeCopyScalarInstruction(Instruction *vectorInstr, Instruction *target);
// i.e vector a = [some operation with scalar b] -> b_vector = cast to vector b; a = [some operation with b_vector]
  Instruction* makeCopyVectorInstruction(Instruction *scalarInstr, llvm::Type *vectorType);
//main function of optimization 
  void addCopyInstruction(std::set<Instruction*> subgraph);
//instead convert vector value and store it, we can use masked vector store
  void optStore(Instruction *vectorInstr);
//instead read scalar value and convert it, we can read vector value
  bool isOptLoad(Instruction *scalarInstr);
  DFG innerDFG;
  Loop *loop;
  LoopInfo *loopInfo;
  DominatorTree *dominatorTree;  
  std::map<std::tuple<Instruction *, BasicBlock *>, Instruction *> copyVectorInstruction;
  std::map<Instruction *, Instruction *> copyScalarInstruction;
  std::map<Value *, Value *> vectorConstant;
  std::map<BasicBlock *, BasicBlock *> supportBlocks;
};

}
#endif

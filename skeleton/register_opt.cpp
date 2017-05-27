#include "llvm/ADT/Statistic.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instruction.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/IR/Dominators.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Support/MapRegIRVreg.hpp"
#include "llvm/Support/CommandLine.h"

#include "iostream"
#include <string>
#include <map>
#include <list>
#include "dfg.hpp"
#include "loop_register_opt.hpp"
#include "llvm/Pass.h"
#include "llvm/IR/InstIterator.h"
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/LoopInfo.h>
#include <llvm/Bitcode/ReaderWriter.h>

#define OUTPUT_FILE "output.ll" 

#define STRINGIFY(X) std::string(#X)
#define DELIM '|'

using namespace llvm;

static cl::opt<std::string> InputInstrs("input-instr", cl::desc("Input instruction which change from scalar to vector"),
    cl::value_desc(std::string("instruction should devides by " + STRINGIFY(DELIM) + " i.e instr1" + STRINGIFY(DELIM) + "instr2").c_str()));

namespace {

struct RegisterOptimization : public ModulePass {
  static char ID; // Pass identification, replacement for typeid
  RegisterOptimization() : ModulePass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<LoopInfoWrapperPass>();
    AU.addRequired<DominatorTreeWrapperPass>();
  }
   	
  std::list<std::string> split(const std::string &s, char delim) {
    std::stringstream ss(s);
    std::string item;
    std::list<std::string> tokens;
    while (getline(ss, item, delim)) {
      tokens.push_back(item);
    }
    return tokens;
	}
	 
  void fillNameToInstr(Function &f){
    for(auto &b : f){
      for(auto &i : b){
        nameToInstr[RegisterOpt::DFG::getInstructionName(&i)] = &i;
      }
    }
  }
  
  void writeToFile(const std::string &filename, Module &M){
    std::string str;
    raw_string_ostream raw_stream(str);
    raw_stream << M;
    std::ofstream myfile;
    myfile.open (filename);
    myfile << raw_stream.str();
    myfile.close();
  }

  std::map<Loop*, std::set<Instruction*>> getloopsWithInstructions(){
    std::map<Loop*, std::set<Instruction*>> loopsWithInstructions;
    std::list<std::string> instrs = split(InputInstrs, '|');
    for(auto &instrName : instrs){							
      Instruction *instr = nameToInstr[instrName];	
      Loop *loop = LI->getLoopFor(instr->getParent());
      if(!loop)
        continue;
      if(loopsWithInstructions.find(loop) == loopsWithInstructions.end())
        loopsWithInstructions[loop] = std::set<Instruction*>();
      loopsWithInstructions[loop].insert(instr);
    }
    return loopsWithInstructions;
  }
   
  bool runOnModule(Module &M) override {
    for(Function &F : M){
      if (F.isDeclaration())
        continue;
      LI = &(getAnalysis<LoopInfoWrapperPass>(F).getLoopInfo());
      DominatorTree *dominatorTree = &(getAnalysis<DominatorTreeWrapperPass>(F).getDomTree());
      fillNameToInstr(F);
      int num = 0;
      auto loopsWithInstructions = getloopsWithInstructions();
      for(auto &loop : loopsWithInstructions){	
        RegisterOpt::LoopRegisterOpt dfg(loop.first, LI, dominatorTree);
        dfg.writeGraph("before"+std::to_string(num)+".dot");
        dfg.makeOpt(loop.second);
        dfg.writeGraph("after"+std::to_string(num++)+".dot");
      }		
    }
    writeToFile(OUTPUT_FILE, M);
    return false;
  }
  private:
    std::map<std::string, Instruction*> nameToInstr;
    LoopInfo *LI;
  };
}

char RegisterOptimization::ID = 0;
static RegisterPass<RegisterOptimization> X("RegisterOptimization", "Enable Vector register as GP",true ,true);

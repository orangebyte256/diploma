#include "dfg.hpp"
#include <climits>
#include <sstream>
#include"llvm/IR/ModuleSlotTracker.h"

namespace RegisterOpt{

std::string getSlotNum(const Instruction *instr){
    int n = 0;
    const Function *M = instr->getParent() ? instr->getParent()->getParent() : nullptr;
    const Module *m =  M ? M->getParent() : nullptr;
    ModuleSlotTracker MST(m, false);
    MST.incorporateFunction(*M);
    n = MST.getLocalSlot((instr));
    return (n == -1) ? "unnamed"+std::to_string((size_t)(instr)) : std::to_string(n);
}

std::string DFG::getInstructionName(const Instruction *instr){
  return std::string((instr->hasName() ? instr->getName().data() : getSlotNum(instr)).c_str());
}



void DFG::writeGraph(const std::string &file){

  std::ofstream myfile;
 	std::map<std::string,std::string> graph_attr, vertex_attr, edge_attr;
	std::vector<std::string> arr;

  myfile.open (file);
  myfile << "digraph G {" << std::endl << "subgraph cluster_0 {" << std::endl;
  std::stringstream tmp;
  bool first = true;
write:
  for(auto &val : vertex_descriptors){
		if(boost::get(&Vertex::insideLoop, graph, val.second) == first)
    {
      myfile << val.second << "[shape=" << (boost::get(&Vertex::type, graph, val.second) == InstrType::Vector ? "box" : "circle")  << ", label=\"" << boost::get(&Vertex::name, graph, val.second) << "\"];" << std::endl;
    }
	}
  if(first){
    myfile << "}" << std::endl;
    first = false;
    goto write;
  }
	boost::write_graphviz(tmp, graph, boost::default_writer(), boost::default_writer());
  for(int i = 0; i < (int)(vertex_descriptors.size() + 3); i++){
    std::string t;
    tmp >> t;
  }
  while(!tmp.eof()){
    std::string t, trash;
    tmp >> t;
    tmp >> trash;
    myfile << t << trash << std::endl;
  }
  
  myfile.close();

}

bool DFG::handleLeafLike(vertex_descriptor vertex, InstructionSet& result){
  boost::graph_traits<BoostGraph>::in_edge_iterator ii, edge_input_end;
  boost::graph_traits<BoostGraph>::out_edge_iterator oi, edge_output_end;
  boost::tie(ii,edge_input_end) = in_edges(vertex, graph);
  boost::tie(oi,edge_output_end) = out_edges(vertex, graph);
  if(++ii != edge_input_end)
    return false;
  if(oi == edge_output_end || (result.find(revert_vertex_descriptors[vertex]) != result.end()))
    return true;
  if(++oi == edge_output_end)
    if(handleLeafLike(source(*oi, graph), result)){
      result.insert(revert_vertex_descriptors[vertex]);
      return true;
    } 
  return false;
}


InstructionSet  DFG::getSubDFG(InstructionSet& val){
  InstructionSet processVertex;
  InstructionSet subgraph;
  processVertex.insert(val.begin(), val.end());
  while(!processVertex.empty()){
    Instruction *cur = *(processVertex.begin());
    processVertex.erase(processVertex.begin());
    subgraph.insert(cur);
    boost::graph_traits<BoostGraph>::vertex_iterator i, end;
    boost::graph_traits<BoostGraph>::in_edge_iterator eii, edge_input_end;
    boost::graph_traits<BoostGraph>::out_edge_iterator eio, edge_output_end;
    for (boost::tie(eii,edge_input_end) = in_edges(vertex_descriptors[cur], graph); eii != edge_input_end; ++eii){
      Instruction *instr = revert_vertex_descriptors[source(*eii, graph)];
      if(subgraph.find(instr) == subgraph.end())
        processVertex.insert(instr);
    }
    for (boost::tie(eio,edge_output_end) = out_edges(vertex_descriptors[cur], graph); eio != edge_output_end; ++eio){
      Instruction *instr = revert_vertex_descriptors[source(*eio, graph)];
      if(subgraph.find(instr) == subgraph.end())
        handleLeafLike(source(*eio, graph), subgraph);
    }
  }
  return subgraph; 
}

bool DFG::addCorrectVertex(Instruction *i, bool insideLoop){
  if(!(dyn_cast<BinaryOperator>(i) || dyn_cast<CastInst>(i) 
        || dyn_cast<CmpInst>(i) || dyn_cast<PHINode>(i)))
    return false;
  addVertex(i, insideLoop);
  return true;
}


DFG DFG::makeInternalDFG(const Loop *loop){
  DFG res;
  for(auto &bb : loop->blocks()){
    for(auto &instr : *bb){
      auto i = dyn_cast<Instruction>(&instr);
      if(!i)
        continue;
      if(!res.addCorrectVertex(&instr, loop->contains(&instr)))
        continue;
      for(auto val = instr.op_begin(); val != instr.op_end(); val++){
        if(auto i = dyn_cast<Instruction>(val)){
          if(!loop->contains(i))
            continue;
          if(res.addCorrectVertex(i, loop->contains(i)))
            res.addEdge(&instr, i); 
        }
      }
    }
  }
  return res;
}

DFG DFG::makeFullDFG(const Loop *loop){
  DFG res;
  for(auto &bb : loop->blocks()){
    for(auto &instr : *bb){
      auto i = dyn_cast<Instruction>(&instr);
      if(!i)
        continue;
      res.addVertex(&instr, loop->contains(&instr));
      for(auto val = instr.op_begin(); val != instr.op_end(); val++){
        if(auto i = dyn_cast<Instruction>(val)){
          res.addVertex(i, loop->contains(i));
          res.addEdge(&instr, i); 
        }
      }
      for(auto val : instr.users()){
        if(auto i = dyn_cast<Instruction>(val)){
          res.addVertex(i, loop->contains(i));
          res.addEdge(i,&instr); 
        }
      }
    }
  }
  return res;
}

void DFG::addVertex(Instruction *val, bool insideLoop){
  if(vertex_descriptors.find(val) == vertex_descriptors.end()){
    vertex_descriptor vertex = add_vertex(graph);
    graph[vertex].name = getInstructionName(val);
    graph[vertex].type = (val->getType()->isVectorTy()) ? InstrType::Vector : InstrType::Scalar;
		graph[vertex].insideLoop = insideLoop;
    vertex_descriptors[val] = vertex;
    revert_vertex_descriptors[vertex] = val;
  }

}

void DFG::addEdge(Instruction *first, Instruction *second){
  if(!std::get<1>(edge( vertex_descriptors[first],vertex_descriptors[second], graph)))
    add_edge(vertex_descriptors[first], vertex_descriptors[second], graph); 
}

}

#ifndef DFG_HPP
#define DFG_HPP

#include "llvm/IR/Instruction.h"
#include <llvm/Analysis/LoopInfo.h>
#include <boost/utility.hpp>     
#include <boost/graph/adjacency_list.hpp>
#include <boost/config.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/subgraph.hpp>
#include <string>
#include <map>

using namespace boost;
using namespace llvm;

using GraphvizAttributes = 
    std::map<std::string, std::string>;
enum InstrType{Scalar, Vector};
struct Vertex{std::string name; InstrType type; bool insideLoop;};

typedef adjacency_list<
vecS,vecS,bidirectionalS, 
  Vertex, 
  boost::property<boost::edge_index_t, std::size_t>,
  boost::property<graph_name_t, std::string,
  boost::property<graph_graph_attribute_t,  GraphvizAttributes,
  boost::property<graph_vertex_attribute_t, Vertex, //>>>> BoostGraph;
	boost::property<boost::edge_index_t, std::size_t > > > > >  BoostGraph;


typedef graph_traits<BoostGraph>::vertex_descriptor vertex_descriptor;


namespace RegisterOpt{
using InstructionSet = std::set<Instruction*>;

class DFG{
  public:
    static DFG makeInternalDFG(const Loop *loop);
    static DFG makeFullDFG(const Loop *loop);
    InstructionSet  getSubDFG(InstructionSet& val);
    void writeGraph(const std::string&);
    static std::string getInstructionName(const Instruction *instr);
  private:
    void addEdge(Instruction *first, Instruction *second);
    void addVertex(Instruction *val, bool insideLoop);
    bool addCorrectVertex(Instruction *val, bool insideLoop);
    bool handleLeafLike(vertex_descriptor vertex, InstructionSet &result);
    BoostGraph graph;
    std::map<Instruction *, vertex_descriptor> vertex_descriptors;
    std::map<vertex_descriptor, Instruction *> revert_vertex_descriptors;
};
}
#endif

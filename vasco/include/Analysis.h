#ifndef ANALYSIS_H
#define ANALYSIS_H

#include "llvm/IR/Function.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Instructions.h"
#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/ADT/iterator_range.h"
#include "llvm/IR/CFG.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/IRBuilder.h"
#include <string>
#include <stack>
#include <map>
#include <set>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <ostream>
#include <unistd.h>
#include <ios>
#include <iomanip>
#include "chrono"

#include "Context.h"
#include "Worklist.h"

using namespace llvm;
using namespace std;
using namespace std::chrono;
enum NoAnalysisType {
    NoAnalyisInThisDirection
};

void process_mem_usage(float &vm_usage, float &resident_set) {
    using std::ios_base;
    using std::ifstream;
    using std::string;

    vm_usage = 0.0;
    resident_set = 0.0;

    // 'file' stat seems to give the most reliable results
    //
    ifstream stat_stream("/proc/self/stat", ios_base::in);

    // dummy vars for leading entries in stat that we don't care about
    //
    string pid, comm, state, ppid, pgrp, session, tty_nr;
    string tpgid, flags, minflt, cminflt, majflt, cmajflt;
    string utime, stime, cutime, cstime, priority, nice;
    string O, itrealvalue, starttime;

    // the two fields we want
    //
    unsigned long vsize;
    long rss;

    stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
                >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
                >> utime >> stime >> cutime >> cstime >> priority >> nice
                >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest

    stat_stream.close();
    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
    vm_usage = static_cast<float>(vsize) / 1024.0f;
    resident_set = static_cast<float>(rss * page_size_kb);
}

void printMemory(float memory) {
    cout << fixed;
    cout << setprecision(6);
    cout << memory / 1024.0;
    cout << " MB\n";
}

class HashFunction {
public:
    auto operator()(const pair<int, BasicBlock *> &P) const {
        // return hash<int>()(P.first) ^ std::hash<BasicBlock *>()(P.second);
        // Same as above line, but viewed the actual source code of C++ and then wrote it
        // Referred: functional_hash.h (line 114)
        // Referred: functional_hash.h (line 110)
        return static_cast<size_t>(P.first) ^ reinterpret_cast<size_t>(P.second);

        // NOTE: Commented this as this will be too slow
        // std::string s1 = std::to_string(P.first);
        // std::string s2 = std::to_string((long) P.second);
        // return stold(s1 + s2);
    }

    auto operator()(const pair<int, Instruction *> &P) const {
        // return hash<int>()(P.first) ^ std::hash<Instruction *>()(P.second);
        // Same as above line, but viewed the actual source code of C++ and then wrote it
        // Referred: functional_hash.h (line 114)
        // Referred: functional_hash.h (line 110)
        return static_cast<size_t>(P.first) ^ reinterpret_cast<size_t>(P.second);

        // NOTE: Commented this as this will be too slow
        // std::string s1 = std::to_string(P.first);
        // std::string s2 = std::to_string((long) P.second);
        // return stold(s1 + s2);
    }
};


// TODO can replace all contexts with context label counter
template<class F, class B>
class Analysis {
private:
    Module *current_module;
    int context_label_counter;
    int current_analysis_direction{}; //0:initial pass, 1:forward, 2:backward
    int processing_context_label{};
    std::unordered_map<int,unordered_map<llvm::Instruction *,pair<F,B>>> IN, OUT;
    std::unordered_map<int, bool> isFree;
    std::string direction;
    unordered_map<int, Context<F,B> *> context_label_to_context_object_map;

    //mapping from context object to context label
    //mapping from function to  pair<inflow,outflow>
    //inflow and outflow are themselves pairs of forward and backward component values.
    //The forward and backward components are themselves pairs of G,L dataflow values.
    bool debug{};
    float total_memory{}, vm{}, rss{};

    void printLine(int);

protected:

    //List of contexts
    unordered_set<int> ProcedureContext;
    Worklist<pair<int,BasicBlock *>,HashFunction> backward_worklist, forward_worklist;

    // mapping from (context label,call site) to target context label
    unordered_map<pair<int, Instruction *>, int, HashFunction> context_transition_graph; //graph
public:
    explicit Analysis(bool);

    Analysis(bool, const string &);

    ~Analysis();

    int getContextLabelCounter();

    void setContextLabelCounter(int);

    int getCurrentAnalysisDirection();

    void setCurrentAnalysisDirection(int);

    int getProcessingContextLabel() const;

    void setProcessingContextLabel(int);

    int getNumberOfContexts();

    void doAnalysis(Module &M);

    void INIT_CONTEXT(llvm::Function *, const std::pair<F, B> &, const std::pair<F, B> &);

    void doAnalysisForward();

    void doAnalysisBackward();

    F NormalFlowFunctionForward(pair<int, BasicBlock *>);

    B NormalFlowFunctionBackward(pair<int, BasicBlock *>);

    int check_if_context_already_exists(llvm::Function *, const pair<F, B> &, const pair<F, B> &);

    void printWorklistMaps();

    bool isAnIgnorableDebugInstruction(Instruction *);

    void performSplittingBB(Function &f);

    void setCurrentModule(Module *);

    Module *getCurrentModule();

    F getForwardComponentAtInOfThisInstruction(Instruction &I);

    B getBackwardComponentAtInOfThisInstruction(Instruction &I);

    F getForwardComponentAtOutOfThisInstruction(Instruction &I);

    B getBackwardComponentAtOutOfThisInstruction(Instruction &I);

    void setForwardComponentAtInOfThisInstruction(Instruction *I, const F& in_value);

    void setBackwardComponentAtInOfThisInstruction(Instruction *I, const B& in_value);

    void setForwardComponentAtOutOfThisInstruction(Instruction *I, const F& out_value);

    void setBackwardComponentAtOutOfThisInstruction(Instruction *I, const B& out_value);

    pair<F, B> getIn(int, llvm::BasicBlock *);

    pair<F, B> getOut(int, llvm::BasicBlock *);

    void setForwardIn(int, llvm::BasicBlock *, const F &);

    void setForwardOut(int, llvm::BasicBlock *, const F &);

    void setBackwardIn(int, llvm::BasicBlock *, const B &);

    void setBackwardOut(int, llvm::BasicBlock *, const B &);


    F getForwardInflowForThisContext(int);

    B getBackwardInflowForThisContext(int);

    F getForwardOutflowForThisContext(int);

    B getBackwardOutflowForThisContext(int);

    void setForwardInflowForThisContext(int, const F &);

    void setBackwardInflowForThisContext(int, const B &);

    void setForwardOutflowForThisContext(int, const F &);

    void setBackwardOutflowForThisContext(int, const B &);

    void printModule(Module &M);

    Function *getFunctionAssociatedWithThisContext(int);

    void printContext();

    virtual pair<F, B> CallInflowFunction(int, Function *, BasicBlock *, const F &, const B &);

    virtual pair<F, B> CallOutflowFunction(int, Function *, BasicBlock *, const F &, const B &, const F &, const B &);


    virtual F computeOutFromIn(Instruction &I);

    virtual F getBoundaryInformationForward();//{}
    virtual F getInitialisationValueForward();//{}
    virtual F performMeetForward(const F& d1, const F& d2) const;//{}
    virtual bool EqualDataFlowValuesForward(const F &d1, const F &d2) const;//{}
    virtual F getPurelyLocalComponentForward(const F& dfv) const;

    virtual F getPurelyGlobalComponentForward(const F& dfv) const;
    virtual F getMixedComponentForward(const F& dfv) const;
    virtual F getCombinedValuesAtCallForward(const F& dfv1,const F& dfv2) const;

    virtual void printDataFlowValuesForward(const F &dfv) const {}

    virtual B computeInFromOut(llvm::Instruction &I);

    virtual B getBoundaryInformationBackward();//{}
    virtual B getInitialisationValueBackward();//{}
    virtual B performMeetBackward(const B& d1, const B& d2) const;//{}
    virtual bool EqualDataFlowValuesBackward(const B& d1, const B& d2) const;//{}
    virtual B getPurelyLocalComponentBackward(const B& dfv) const;
    virtual B getPurelyGlobalComponentBackward(const B& dfv) const;
    virtual B getMixedComponentBackward(const B& dfv) const;
    virtual B getCombinedValuesAtCallBackward(const B& dfv1, const B& dfv2) const;

    virtual void printDataFlowValuesBackward(const B& dfv) const {}


};

//========================================================================================
template<class F, class B>
void Analysis<F, B>::printLine(int label) {
    string Name = "";
    if (current_analysis_direction == 1) {
        Name = "FORWARD";
    } else {
        Name = "BACKWARD";
    }
    std::string Str = "\n===================================[" + Name + "-" + to_string(label) +
                      " ]===========================================\n";
    llvm::outs() << Str;
}


template<class F, class B>
Analysis<F, B>::Analysis(bool debug) {
    current_module = nullptr;
    context_label_counter = -1;
    this->debug = debug;
    this->direction = "";
}

template<class F, class B>
Analysis<F, B>::Analysis(bool debug, const string &fileName) {
    current_module = nullptr;
    context_label_counter = -1;
    this->debug = debug;
    freopen(fileName.c_str(), "w", stdout);
    this->direction = "";
}

template<class F, class B>
Analysis<F, B>::~Analysis() {
    llvm::outs() << "Memory consume: ";
    printMemory(this->total_memory);
}

template<class F, class B>
int Analysis<F, B>::getContextLabelCounter() {
    return context_label_counter;
}

template<class F, class B>
void Analysis<F, B>::setContextLabelCounter(int new_context_label_counter) {
    context_label_counter = new_context_label_counter;
}

template<class F, class B>
int Analysis<F, B>::getCurrentAnalysisDirection() {
    return current_analysis_direction;
}

template<class F, class B>
void Analysis<F, B>::setCurrentAnalysisDirection(int direction) {
    current_analysis_direction = direction;
}

template<class F, class B>
int Analysis<F, B>::getProcessingContextLabel() const {
    return processing_context_label;
}

template<class F, class B>
void Analysis<F, B>::setProcessingContextLabel(int label) {
    processing_context_label = label;
}

template<class F, class B>
bool Analysis<F, B>::isAnIgnorableDebugInstruction(Instruction *inst) {
    if (isa<DbgDeclareInst>(inst) || isa<DbgValueInst>(inst)) {
        return true;
    }
    return false;
}

template<class F, class B>
Module *Analysis<F, B>::getCurrentModule() {
    return current_module;
}

template<class F, class B>
void Analysis<F, B>::setCurrentModule(Module *m) {
    current_module = m;
}

template<class F, class B>
B Analysis<F, B>::computeInFromOut(Instruction &I) {
    llvm::outs() << "\nThis function computeInFromOut() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F, class B>
F Analysis<F, B>::computeOutFromIn(Instruction &I) {
    llvm::outs() << "\nThis function computeOutFromIn() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F, class B>
F Analysis<F, B>::getBoundaryInformationForward() {
    llvm::outs() << "\nThis function getBoundaryInformationForward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F, class B>
B Analysis<F, B>::getBoundaryInformationBackward() {
    llvm::outs() << "\nThis function getBoundaryInformationBackward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F, class B>
F Analysis<F, B>::getInitialisationValueForward() {
    llvm::outs() << "\nThis function getInitialisationValueForward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F, class B>
B Analysis<F, B>::getInitialisationValueBackward() {
    llvm::outs() << "\nThis function getInitialisationValueBackward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F, class B>
F Analysis<F, B>::performMeetForward(const F& d1, const F& d2) const {
    llvm::outs() << "\nThis function performMeetForward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F, class B>
B Analysis<F, B>::performMeetBackward(const B& d1, const B& d2) const {
    llvm::outs() << "\nThis function performMeetBackward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F, class B>
bool Analysis<F, B>::EqualDataFlowValuesForward(const F &d1, const F &d2) const {
    llvm::outs() << "\nThis function EqualDataFlowValuesForward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F, class B>
bool Analysis<F, B>::EqualDataFlowValuesBackward(const B& d1, const B& d2) const {
    llvm::outs() << "\nThis function EqualDataFlowValuesBackward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F, class B>
B Analysis<F, B>::getPurelyGlobalComponentBackward(const B& dfv) const {
    llvm::outs() << "\nThis function getPurelyGlobalComponentBackward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F, class B>
F Analysis<F, B>::getPurelyGlobalComponentForward(const F& dfv) const {
    llvm::outs() << "\nThis function getPurelyGlobalComponentForward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F, class B>
F Analysis<F, B>::getPurelyLocalComponentForward(const F& dfv) const {
    llvm::outs() << "\nThis function getPurelyLocalComponentForward() has not been implemented. EXITING !!\n";
    exit(-1);
}
template <class F,class B>
B Analysis<F,B>::getPurelyLocalComponentBackward(const B& dfv) const
{
    llvm::outs()<<"\nThis function getPurelyLocalComponentBackward() has not been implemented. EXITING !!\n";
    exit(-1);  
}



template <class F,class B>
B Analysis<F,B>::getMixedComponentBackward(const B& dfv) const
{
    llvm::outs()<<"\nThis function getMixedComponentBackward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
F Analysis<F,B>::getMixedComponentForward(const F& dfv) const
{
    llvm::outs()<<"\nThis function getMixedComponentForward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
B Analysis<F,B>::getCombinedValuesAtCallBackward(const B& dfv1, const B& dfv2) const
{
    llvm::outs()<<"\nThis function getCombinedValuesAtCallBackward() has not been implemented. EXITING !!\n";
    exit(-1);
}

template <class F,class B>
F Analysis<F,B>::getCombinedValuesAtCallForward(const F& dfv1, const F& dfv2) const
{
    llvm::outs()<<"\nThis function getCombinedValuesAtCallForward() has not been implemented. EXITING !!\n";
    exit(-1);
}
//========================================================================================

template<class F, class B>
void Analysis<F, B>::printModule(Module &M) {
    llvm::outs() << "--------------------------------------------" << "\n";
    for (Function &Func : M) {
        llvm::outs() << "FUNCTION NAME: ";
        llvm::outs() << Func.getName() << "\n";
        llvm::outs() << "----------------------------" << "\n";
        for (BasicBlock &BB : Func) {
            llvm::outs() << "----------------------" << "\n";
            for (Instruction &inst : BB) {
                llvm::outs() << inst << "\n";
            }
            llvm::outs() << "----------------------" << "\n";
        }
        llvm::outs() << "-----------------------------" << "\n";
    }
    llvm::outs() << "---------------------------------------------" << "\n";
}


template<class F, class B>
pair<F, B>
Analysis<F, B>::CallInflowFunction(int context_label, Function *target_function, BasicBlock *bb, const F& a1, const B& d1) {
    llvm::outs() << "\nThis function CallInflowFunction() has not been implemented. EXITING !!\n";
    exit(-1);
}

template<class F, class B>
pair<F, B>
Analysis<F, B>::CallOutflowFunction(int context_label, Function *target_function, BasicBlock *bb, const F& a3, const B& d3, const F& a1,
                                    const B& d1) {
    llvm::outs() << "\nThis function CallOutflowFunction() has not been implemented. EXITING !!\n";
    exit(-1);
}

//=====================setter and getters for IN-OUT Maps==================================
template<class F, class B>
F Analysis<F, B>::getForwardComponentAtInOfThisInstruction(Instruction &I) {
    int label = getProcessingContextLabel();
    return IN[label][&I].first;
}

template<class F, class B>
F Analysis<F, B>::getForwardComponentAtOutOfThisInstruction(Instruction &I) {
    int label = getProcessingContextLabel();
    return OUT[label][&I].first;
}

template<class F, class B>
B Analysis<F, B>::getBackwardComponentAtInOfThisInstruction(Instruction &I) {
    int label = getProcessingContextLabel();
    return IN[label][&I].second;
}

template<class F, class B>
B Analysis<F, B>::getBackwardComponentAtOutOfThisInstruction(Instruction &I) {
    int label = getProcessingContextLabel();
    return OUT[label][&I].second;
}

template<class F, class B>
void Analysis<F, B>::setForwardComponentAtInOfThisInstruction(Instruction *I,const F& in_value) {
    int label = getProcessingContextLabel();
    IN[label][I].first = in_value;
}

template<class F, class B>
void Analysis<F, B>::setForwardComponentAtOutOfThisInstruction(Instruction *I, const F& out_value) {
    int label = getProcessingContextLabel();
    OUT[label][I].first = out_value;
}

template<class F, class B>
void Analysis<F, B>::setBackwardComponentAtInOfThisInstruction(Instruction *I, const B& in_value) {
    int label = getProcessingContextLabel();
    IN[label][I].second = in_value;
}

template<class F, class B>
void Analysis<F, B>::setBackwardComponentAtOutOfThisInstruction(Instruction *I, const B& out_value) {
    int label = getProcessingContextLabel();
    OUT[label][I].second = out_value;
}

//=====================setter and getters CS_BB==================================

template<class F, class B>
pair<F, B> Analysis<F, B>::getIn(int label, llvm::BasicBlock *BB) {
//    return IN[{label,&(*BB->begin())}];
    return IN[label][&(*(BB->begin()))];
}

template<class F, class B>
pair<F, B> Analysis<F, B>::getOut(int label, llvm::BasicBlock *BB) {
    return OUT[label][&(BB->back())];
}

template<class F, class B>
void Analysis<F, B>::setForwardIn(int label, llvm::BasicBlock *BB, const F& dataflowvalue) {
    IN[label][&(*(BB->begin()))].first = dataflowvalue;
}

template<class F, class B>
void Analysis<F, B>::setForwardOut(int label, llvm::BasicBlock *BB, const F& dataflowvalue) {
    OUT[label][&(BB->back())].first = dataflowvalue;
}

template<class F, class B>
void Analysis<F, B>::setBackwardIn(int label, llvm::BasicBlock *BB, const B& dataflowvalue) {
    IN[label][&(*(BB->begin()))].second = dataflowvalue;
}

template<class F, class B>
void Analysis<F, B>::setBackwardOut(int label, llvm::BasicBlock *BB, const B& dataflowvalue) {
    OUT[label][&(BB->back())].second = dataflowvalue;
}

//=================================================================================




//=====================setter and getters for context objects==================================
template<class F, class B>
F Analysis<F, B>::getForwardInflowForThisContext(int context_label) {
//    return context_label_to_context_object_map[context_label].second.first.first;
    return context_label_to_context_object_map[context_label]->getInflowValue().first;
}

template<class F, class B>
B Analysis<F, B>::getBackwardInflowForThisContext(int context_label) {
//    return context_label_to_context_object_map[context_label].second.first.second;
    return context_label_to_context_object_map[context_label]->getInflowValue().second;
}

template<class F, class B>
F Analysis<F, B>::getForwardOutflowForThisContext(int context_label) {
//    return context_label_to_context_object_map[context_label].second.second.first;
    return context_label_to_context_object_map[context_label]->getOutflowValue().first;
}

template<class F, class B>
B Analysis<F, B>::getBackwardOutflowForThisContext(int context_label) {
//    return context_label_to_context_object_map[context_label].second.second.second;
    return context_label_to_context_object_map[context_label]->getOutflowValue().second;
}


template<class F, class B>
void Analysis<F, B>::setForwardInflowForThisContext(int context_label, const F& forward_inflow) {
//    context_label_to_context_object_map[context_label].second.first.first=forward_inflow;
    context_label_to_context_object_map[context_label]->setForwardInflow(forward_inflow);
}

template<class F, class B>
void Analysis<F, B>::setBackwardInflowForThisContext(int context_label, const B& backward_inflow) {
//    context_label_to_context_object_map[context_label].second.first.second=backward_inflow;
    context_label_to_context_object_map[context_label]->setBackwardInflow(backward_inflow);
}

template<class F, class B>
void Analysis<F, B>::setForwardOutflowForThisContext(int context_label, const F& forward_outflow) {
//    context_label_to_context_object_map[context_label].second.second.first=forward_outflow;
    context_label_to_context_object_map[context_label]->setForwardOutflow(forward_outflow);
}

template<class F, class B>
void Analysis<F, B>::setBackwardOutflowForThisContext(int context_label, const B& backward_outflow) {
//    context_label_to_context_object_map[context_label].second.second.second=backward_outflow;
    context_label_to_context_object_map[context_label]->setBackwardOutflow(backward_outflow);
}

template<class F, class B>
Function *Analysis<F, B>::getFunctionAssociatedWithThisContext(int context_label) {
    return context_label_to_context_object_map[context_label]->getFunction();
}

template<class F, class B>
int Analysis<F, B>::getNumberOfContexts() {
    return ProcedureContext.size();
}

//================================================================================================
template<class F, class B>
void Analysis<F, B>::doAnalysis(Module &M) {
    setCurrentModule(&M);
    // ======================================================================================

    //====================================SPLITTING========================================
    auto start = high_resolution_clock::now();
    for (Function &function: M) {
        if (function.size() > 0) {
            performSplittingBB(function);
        }
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<seconds>(stop - start);
    llvm::outs() << "Time taken in Splitting Basic Block : " << duration.count() << " seconds" << "\n";
    start = high_resolution_clock::now();
    int i = 0;
    for (Function &function: M) {
        if (function.getName() == "main") {
            F forward_inflow_bi;//=getBoundaryInformationForward();
            B backward_inflow_bi;//=getBoundaryInformationBackward();
            F forward_outflow_bi;
            B backward_outflow_bi;
            Function *fptr = &function;
            if (std::is_same<F, NoAnalysisType>::value) {
                // forward_bi=NoAnalyisInThisDirection;   
            } else {
                forward_inflow_bi = getBoundaryInformationForward();
            }
            if (std::is_same<B, NoAnalysisType>::value) {
                // forward_bi=NoAnalyisInThisDirection;   
            } else {
                backward_inflow_bi = getBoundaryInformationBackward();
            }
            setCurrentAnalysisDirection(0);
            INIT_CONTEXT(fptr, {forward_inflow_bi, backward_inflow_bi}, {forward_outflow_bi, backward_outflow_bi});
//            INIT_CONTEXT(make_pair(fptr,make_pair(make_pair(forward_inflow_bi,backward_inflow_bi),make_pair(forward_outflow_bi,backward_outflow_bi))));
        }
    }
    if (std::is_same<F, NoAnalysisType>::value) {
        //backward analysis
        direction = "backward";
        setCurrentAnalysisDirection(2);
        while (not backward_worklist.empty()) {
            doAnalysisBackward();
        }
    } else if (std::is_same<B, NoAnalysisType>::value) {
        //forward analysis
        direction = "forward";
        setCurrentAnalysisDirection(1);
        while (not forward_worklist.empty()) {
            doAnalysisForward();
        }
    } else {
        direction = "bidirectional";
        int fi=1,bi=1;
        int iteration = 1;
        while (forward_worklist.size()>0 || backward_worklist.size()>0)
        {
            // current_analysis_direction=2;
            setCurrentAnalysisDirection(2);
            while(backward_worklist.size()>0)
            {
                doAnalysisBackward();
            }
            // current_analysis_direction=1;
            setCurrentAnalysisDirection(1);
            while(forward_worklist.size()>0)
            {
                doAnalysisForward();
            }
            iteration++;
        }
    }
    stop = high_resolution_clock::now();
    duration = duration_cast<seconds>(stop - start);
    llvm::outs() << "Time taken by analysis: " << duration.count() << " seconds" << "\n";
}


template<class F, class B>
void Analysis<F, B>::INIT_CONTEXT(llvm::Function *function, const std::pair<F, B>& Inflow, const std::pair<F, B>& Outflow) {
    context_label_counter++;
    Context<F,B> *context_object = new Context<F,B>(context_label_counter,function,Inflow,Outflow);
    int current_context_label = context_object->getLabel();
    setProcessingContextLabel(current_context_label);
    if (std::is_same<B, NoAnalysisType>::value) {
        if (debug) {
            llvm::outs() << "INITIALIZING CONTEXT:-" << "\n";
            llvm::outs() << "LABEL: " << context_object->getLabel() << "\n";
            llvm::outs() << "FUNCTION: " << function->getName() << "\n";
            llvm::outs() << "Inflow Value: ";
            printDataFlowValuesForward(Inflow.first);
            llvm::outs() << "Outflow value: ";
            printDataFlowValuesForward(getInitialisationValueForward());
        }
        //forward analysis
        context_label_to_context_object_map[current_context_label] = context_object;

        setForwardOutflowForThisContext(current_context_label,
                                        getInitialisationValueForward());//setting outflow forward
        ProcedureContext.insert(current_context_label);

        for (BasicBlock *BB:post_order(&context_object->getFunction()->getEntryBlock())) {
            BasicBlock &b = *BB;
            forward_worklist.workInsert({current_context_label,&b});
//            forward_worklist.push(make_pair(current_context_label, &b));
//            forward_worklist_contains_this_entry[make_pair(current_context_label, &b)] = true;
            if(direction == "bidirectional"){
                backward_worklist.workInsert({current_context_label,&b});
//                backward_worklist.push(make_pair(current_context_label, &b));
//                backward_worklist_contains_this_entry[make_pair(current_context_label, &b)] = true;
            }
            setForwardIn(current_context_label, &b, getInitialisationValueForward());
            setForwardOut(current_context_label, &b, getInitialisationValueForward());
//            CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//            CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();

            //initialise IN-OUT maps for every instruction
            for (auto inst = &*(b.begin()); inst != nullptr; inst = inst->getNextNonDebugInstruction()) {
                setForwardComponentAtInOfThisInstruction(&(*inst), getInitialisationValueForward());
                setForwardComponentAtOutOfThisInstruction(&(*inst), getInitialisationValueForward());
            }
        }
        if (current_context_label == 0)//main function with first invocation
        {
            setForwardInflowForThisContext(current_context_label,
                                           getBoundaryInformationForward());//setting inflow forward
        } else {
//            setForwardInflowForThisContext(current_context_label,context_object.second.first.first);//setting inflow forward
            setForwardInflowForThisContext(current_context_label, context_object->getInflowValue().first);
        }
        setForwardIn(current_context_label, &context_object->getFunction()->getEntryBlock(),
                     getForwardInflowForThisContext(current_context_label));
//        CS_BB_IN[make_pair(current_context_label,&context_object.getFunction()->getEntryBlock())].first=getForwardInflowForThisContext(current_context_label);
    } else if (std::is_same<F, NoAnalysisType>::value) {
        if (debug) {
            llvm::outs() << "INITIALIZING CONTEXT:-" << "\n";
            llvm::outs() << "LABEL: " << context_object->getLabel() << "\n";
            llvm::outs() << "FUNCTION: " << function->getName() << "\n";
            llvm::outs() << "Inflow Value: ";
            printDataFlowValuesBackward(Inflow.second);
        }
        //backward analysis
        context_label_to_context_object_map[current_context_label] = context_object;
        setBackwardOutflowForThisContext(current_context_label,
                                         getInitialisationValueBackward());//setting outflow backward
        ProcedureContext.insert(current_context_label);

        for (BasicBlock *BB:inverse_post_order(&context_object->getFunction()->back())) {
            BasicBlock &b = *BB;
            backward_worklist.workInsert(make_pair(current_context_label, &b));
            if(direction == "bidirectional"){
                backward_worklist.workInsert(make_pair(current_context_label, &b));
            }
            setBackwardIn(current_context_label, &b, getInitialisationValueBackward());
            setBackwardOut(current_context_label, &b, getInitialisationValueBackward());
//            CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//            CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();

            //initialise IN-OUT maps for every instruction
            for (auto inst = &*(b.begin()); inst != nullptr; inst = inst->getNextNonDebugInstruction()) {
                setBackwardComponentAtInOfThisInstruction(&(*inst), getInitialisationValueBackward());
                setBackwardComponentAtOutOfThisInstruction(&(*inst), getInitialisationValueBackward());
            }
        }
        if (current_context_label == 0)//main function with first invocation
        {
            setBackwardInflowForThisContext(current_context_label,
                                            getBoundaryInformationBackward());//setting inflow backward
        } else {
            setBackwardInflowForThisContext(current_context_label,
                                            context_object->getInflowValue().second);//setting inflow backward
        }
        setBackwardOut(current_context_label, &context_object->getFunction()->back(),
                       getBackwardInflowForThisContext(current_context_label));
        setBackwardOut(current_context_label,&context_object->getFunction()->back(), getBackwardInflowForThisContext(current_context_label));
//        CS_BB_OUT[make_pair(current_context_label,&context_object.getFunction().back())].second=getBackwardInflowForThisContext(current_context_label);
    } else {
        // Todo for Bidirectional Analysis
        context_label_to_context_object_map[current_context_label] = context_object;
        if(getCurrentAnalysisDirection() == 10){
            setForwardOutflowForThisContext(current_context_label,getInitialisationValueForward());
            setBackwardOutflowForThisContext(current_context_label,getInitialisationValueBackward());
            ProcedureContext.insert(current_context_label);
            for(BasicBlock *BB : post_order(&context_object->getFunction()->getEntryBlock()))
            {
                BasicBlock &b = *BB;
                // llvm::outs()<<"\nBasicBlock: ";
                // b.printAsOperand(llvm::outs(),false);
//                forward_worklist.push(make_pair(current_context_label,&b));
                forward_worklist.workInsert(make_pair(current_context_label,&b));
                // backward_worklist.push(make_pair(current_context_label,&b));
//                forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
                // backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
                setForwardIn(current_context_label,&b,getInitialisationValueForward());
                setBackwardIn(current_context_label,&b,getInitialisationValueBackward());
                setForwardOut(current_context_label,&b,getInitialisationValueForward());
                setBackwardOut(current_context_label,&b,getInitialisationValueBackward());
//                CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//                CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();

                //initialise IN-OUT maps for every instruction
                for(auto inst=&*(b.begin());inst!=nullptr;inst=inst->getNextNonDebugInstruction())
                {
                    // llvm::outs()<<"\n"<<*inst;
                    setBackwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueBackward());
                    setBackwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueBackward());
                    setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
                    setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
                    // IN[&(*inst)].first=getInitialisationValueForward();
                    // OUT[&(*inst)].first=getInitialisationValueForward();
                    // IN[&(*inst)].second=getInitialisationValueBackward();
                    // OUT[&(*inst)].second=getInitialisationValueBackward();
                }
            }
        } else if(getCurrentAnalysisDirection()==20) {
            setForwardOutflowForThisContext(current_context_label,getInitialisationValueForward());//setting outflow forward
            setBackwardOutflowForThisContext(current_context_label,getInitialisationValueBackward());//setting outflow backward
            // context_label_to_context_object_map[current_context_label].second.second.first=getInitialisationValueForward();//setting outflow forward
            // context_label_to_context_object_map[current_context_label].second.second.second=getInitialisationValueBackward();//setting outflow backward
            ProcedureContext.insert(current_context_label);
            for(BasicBlock *BB : inverse_post_order(&context_object->getFunction()->back()))
            {
                BasicBlock &b=*BB;
                // llvm::outs()<<"\nBasicBlock: ";
                // b.printAsOperand(llvm::outs(),false);
                // forward_worklist.push(make_pair(current_context_label,&b));
                backward_worklist.workInsert(make_pair(current_context_label,&b));
//                backward_worklist.push(make_pair(current_context_label,&b));
                // forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//                backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
                setForwardIn(current_context_label,&b,getInitialisationValueForward());
                setBackwardIn(current_context_label,&b,getInitialisationValueBackward());
                setForwardOut(current_context_label,&b,getInitialisationValueForward());
                setBackwardOut(current_context_label,&b,getInitialisationValueBackward());
//                CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//                CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();

                //initialise IN-OUT maps for every instruction
                for(auto inst=&*(b.begin());inst!=nullptr;inst=inst->getNextNonDebugInstruction())
                {
                    // llvm::outs()<<"\n"<<*inst;
                    setBackwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueBackward());
                    setBackwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueBackward());
                    setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
                    setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
                    // IN[&(*inst)].first=getInitialisationValueForward();
                    // OUT[&(*inst)].first=getInitialisationValueForward();
                    // IN[&(*inst)].second=getInitialisationValueBackward();
                    // OUT[&(*inst)].second=getInitialisationValueBackward();
                }
            }
        } else {
            setForwardOutflowForThisContext(current_context_label,getInitialisationValueForward());//setting outflow forward
            setBackwardOutflowForThisContext(current_context_label,getInitialisationValueBackward());//setting outflow backward
            // context_label_to_context_object_map[current_context_label].second.second.second=getInitialisationValueBackward();//setting outflow backward
            // context_label_to_context_object_map[current_context_label].second.second.first=getInitialisationValueForward();//setting outflow forward
//            context_object_to_context_label_map[context_label_to_context_object_map[current_context_label]]=current_context_label;
            ProcedureContext.insert(current_context_label);
            for(BasicBlock *BB:inverse_post_order(&context_object->getFunction()->back()))
            {
                //populate backward worklist
                BasicBlock &b=*BB;
                // llvm::outs()<<"\nBasicBlock: ";
                // b.printAsOperand(llvm::outs(),false);
                // forward_worklist.push(make_pair(current_context_label,&b));
//                backward_worklist.push(make_pair(current_context_label,&b));
                backward_worklist.workInsert(make_pair(current_context_label,&b));
                // forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//                backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
                // CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
                setBackwardIn(current_context_label,&b,getInitialisationValueBackward());
                // CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
//                CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
                setBackwardOut(current_context_label,&b,getInitialisationValueBackward());

                //initialise IN-OUT maps for every instruction
                for(auto inst=&*(b.begin());inst!=nullptr;inst=inst->getNextNonDebugInstruction())
                {
                    // llvm::outs()<<"\n"<<*inst;
                    setBackwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueBackward());
                    setBackwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueBackward());
                    // setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
                    // setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
                    // IN[&(*inst)].first=getInitialisationValueForward();
                    // OUT[&(*inst)].first=getInitialisationValueForward();
                    // IN[&(*inst)].second=getInitialisationValueBackward();
                    // OUT[&(*inst)].second=getInitialisationValueBackward();
                }
            }
            for(BasicBlock *BB:post_order(&context_object->getFunction()->getEntryBlock()))
            {
                //populate forward worklist
                BasicBlock &b=*BB;
                // llvm::outs()<<"\nBasicBlock: ";
                // b.printAsOperand(llvm::outs(),false);
//                forward_worklist.push(make_pair(current_context_label,&b));
                forward_worklist.workInsert(make_pair(current_context_label,&b));
                // backward_worklist.push(make_pair(current_context_label,&b));
//                forward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
                // backward_worklist_contains_this_entry[make_pair(current_context_label,&b)]=true;
//                CS_BB_IN[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
                setForwardIn(current_context_label,&b,getInitialisationValueForward());
                // CS_BB_IN[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();
//                CS_BB_OUT[make_pair(current_context_label,&b)].first=getInitialisationValueForward();
                setForwardOut(current_context_label,&b,getInitialisationValueForward());
                // CS_BB_OUT[make_pair(current_context_label,&b)].second=getInitialisationValueBackward();

                //initialise IN-OUT maps for every instruction
                for(auto inst=&*(b.begin());inst!=nullptr;inst=inst->getNextNonDebugInstruction())
                {
                    // llvm::outs()<<"\n"<<*inst;
                    setForwardComponentAtInOfThisInstruction(&(*inst),getInitialisationValueForward());
                    setForwardComponentAtOutOfThisInstruction(&(*inst),getInitialisationValueForward());
                    // IN[&(*inst)].first=getInitialisationValueForward();
                    // OUT[&(*inst)].first=getInitialisationValueForward();

                    // IN[&(*inst)].second=getInitialisationValueBackward();
                    // OUT[&(*inst)].second=getInitialisationValueBackward();
                }
            }
            if(current_context_label==0){ //main function with first invocation
                setForwardInflowForThisContext(current_context_label,getBoundaryInformationForward());//setting inflow forward
                setBackwardInflowForThisContext(current_context_label,getBoundaryInformationBackward());//setting inflow backward
            } else {
                setForwardInflowForThisContext(current_context_label,context_object->getInflowValue().first);//setting inflow forward
                setBackwardInflowForThisContext(current_context_label,context_object->getInflowValue().second);//setting inflow backward
            }
            setForwardIn(current_context_label,&context_object->getFunction()->getEntryBlock(),getForwardInflowForThisContext(current_context_label));
            setBackwardOut(current_context_label,&context_object->getFunction()->back(),getBackwardInflowForThisContext(current_context_label));
//        CS_BB_IN[make_pair(current_context_label,&function.getEntryBlock())].first=getForwardInflowForThisContext(current_context_label);
//        CS_BB_OUT[make_pair(current_context_label,&function.back())].second=getBackwardInflowForThisContext(current_context_label);
        }
    }
    process_mem_usage(this->vm, this->rss);
    this->total_memory = max(this->total_memory, this->vm);
}

template<class F, class B>
void Analysis<F, B>::doAnalysisForward() {
    while (not forward_worklist.empty())//step 2
    {
        //step 3 and 4
        pair<int, BasicBlock *> current_pair = forward_worklist.workDelete();
        int current_context_label = current_pair.first;
        BasicBlock *bb = current_pair.second;
        setProcessingContextLabel(current_context_label);

        BasicBlock &b = *bb;
        Function *f = context_label_to_context_object_map[current_context_label]->getFunction();
        Function &function = *f;

        //step 5
        if (bb != (&function.getEntryBlock())) {
            //step 6
            setForwardIn(current_pair.first, current_pair.second, getInitialisationValueForward());
//            CS_BB_IN[current_pair].first=getInitialisationValueForward();

            //step 7 and 8
            for (auto pred_bb:predecessors(bb)) {
                setForwardIn(current_pair.first, current_pair.second,
                             performMeetForward(getIn(current_pair.first, current_pair.second).first,
                                                getOut(current_pair.first,
                                                       pred_bb).first)); // CS_BB_OUT[make_pair(current_pair.first,pred_bb)].first
//                CS_BB_IN[current_pair].first=performMeetForward(CS_BB_IN[current_pair].first,CS_BB_OUT[make_pair(current_pair.first,pred_bb)].first);
            }
        } else {
            //In value of this node is same as INFLOW value
            setForwardIn(current_pair.first, current_pair.second,
                         getForwardInflowForThisContext(current_context_label));
//            CS_BB_IN[current_pair].first=getForwardInflowForThisContext(current_context_label);
        }

        //step 9
        F a1 = getIn(current_pair.first, current_pair.second).first; //CS_BB_IN[current_pair].first;
        if (debug) {
            printLine(current_pair.first);
            llvm::outs() << "TESTING\n";
            llvm::outs() << "INFLOW VALUES: ";
            printDataFlowValuesForward(a1);
            printLine(current_pair.first);
        }
        B d1 = getOut(current_pair.first, current_pair.second).second; //CS_BB_OUT[current_pair].second;

        F previous_value_at_out_of_this_node = getOut(current_pair.first,
                                                      current_pair.second).first; //CS_BB_OUT[current_pair].first;

        //step 10
        bool contains_a_method_call = false;
        for (auto inst = &*(b.begin()); inst != nullptr; inst = inst->getNextNonDebugInstruction()) {
            Instruction &I = *inst;
            if (CallInst *ci = dyn_cast<CallInst>(&I)) {
                Function *target_function = ci->getCalledFunction();
                if (not target_function || target_function->isDeclaration() || isAnIgnorableDebugInstruction(&I)) {
                    continue; //this is an inbuilt function so doesn't need to be processed.
                }
                contains_a_method_call = true;
                break;
            }
        }
        if (contains_a_method_call) {
            F prev = getForwardComponentAtInOfThisInstruction(*(b.begin()));
//            F prev = getIn(current_pair.first,current_pair.second).first; //CS_BB_IN[current_pair].first; //a1
            //step 11
            for (auto inst = &*(b.begin()); inst != nullptr; inst = inst->getNextNonDebugInstruction()) {
                Instruction &I = *inst;
                if (CallInst *ci = dyn_cast<CallInst>(&I)) {
                    Function *target_function = ci->getCalledFunction();
                    if (not target_function || target_function->isDeclaration() || isAnIgnorableDebugInstruction(&I)) {
                        continue; //this is an inbuilt function so doesn't need to be processed.
                    }

                    /*
                    At the call instruction, the value at IN should be splitted into two components:
                    1) Purely Global and 2) Mixed.
                    The purely global component is given to the start of callee.
                    */

                    //step 12
                    pair<F, B> inflow_pair = CallInflowFunction(current_context_label, target_function, bb, a1, d1);
                    F a2 = inflow_pair.first;
                    B d2 = inflow_pair.second;

                    F new_outflow_forward;
                    B new_outflow_backward;

                    //step 13
//                    pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object=make_pair(target_function,make_pair(make_pair(a2,d2),make_pair(new_outflow_forward,new_outflow_backward)));

                    setForwardComponentAtInOfThisInstruction(&(*inst), prev);//compute IN from previous OUT-value
                    int matching_context_label = 0;
//                    matching_context_label = check_if_context_already_exists(new_context_object);
                    matching_context_label = check_if_context_already_exists(target_function, inflow_pair,
                                                                             make_pair(new_outflow_forward,
                                                                                       new_outflow_backward));
                    if (matching_context_label > 0)//step 15
                    {
                        if (debug) {
                            printLine(current_context_label);
                            llvm::outs() << *inst << "\n";
                            llvm::outs() << "IN: ";
                        }
                        //step 14
                        pair<int, Instruction *> mypair = make_pair(current_context_label, &(*inst));
                        context_transition_graph[mypair] = matching_context_label;
                        if (debug) {
                            printDataFlowValuesForward(a1);
                        }

                        //step 16 and 17
                        F a3 = getForwardOutflowForThisContext(matching_context_label);
                        B d3 = getBackwardOutflowForThisContext(matching_context_label);

                        pair<F, B> outflow_pair = CallOutflowFunction(current_context_label, target_function, bb, a3,
                                                                      d3, a1, d1);
                        F value_to_be_meet_with_prev_out = outflow_pair.first;
                        B d4 = outflow_pair.second;

                        //step 18 and 19

                        /*
                        At the call instruction, the value at IN should be splitted into two components.
                        The purely global component is given to the callee while the mixed component is propagated
                        to OUT of this instruction after executing computeOUTfromIN() on it.
                        */

                        F a5 = getPurelyLocalComponentForward(a1);
                        /*
                        At the OUT of this instruction, the value from END of callee procedure is to be merged
                        with the local(mixed) value propagated from IN. Note that this merging "isn't" 
                        exactly (necessarily) the meet between these two values.
                        */


                        /*
                        As explained in ip-vasco,pdf, we need to perform meet with the original value of OUT
                        of this instruction to avoid the oscillation problem.
                        */
                        setForwardComponentAtOutOfThisInstruction(&(*inst), performMeetForward(
                                performMeetForward(value_to_be_meet_with_prev_out,
                                                   getForwardComponentAtOutOfThisInstruction(*inst)), a5));
                        prev = getForwardComponentAtOutOfThisInstruction((*inst));
                        if (debug) {
                            llvm::outs() << "OUT: ";
                            printDataFlowValuesForward(prev);
                            printLine(current_context_label);
                        }
//                        if(matching_context_label != current_context_label){
//                            freeMemory(matching_context_label);
//                        }
                    } else//step 20
                    {
                        //creating a new context
//                        INIT_CONTEXT(new_context_object); //step 21
                        INIT_CONTEXT(target_function, {a2, d2}, {new_outflow_forward, new_outflow_backward}); //step 21

                        //step 14
                        //This step must be done after above step because context label counter gets updated after INIT-Context is invoked.
                        pair<int, Instruction *> mypair = make_pair(current_context_label, &(*inst));
                        context_transition_graph[mypair] = getContextLabelCounter();
                        return;
                    }
                } else {
                    if (debug) {
                        printLine(current_context_label);
                        llvm::outs() << *inst << "\n";
                        llvm::outs() << "IN: ";
                        printDataFlowValuesForward(prev);
                    }
                    setForwardComponentAtInOfThisInstruction(&(*inst), prev);//compute IN from previous OUT-value
                    F new_prev = computeOutFromIn(*inst);
                    setForwardComponentAtOutOfThisInstruction(&(*inst), new_prev);
                    if (debug) {
                        llvm::outs() << "OUT: ";
                        printDataFlowValuesForward(new_prev);
                    }
                    prev = getForwardComponentAtOutOfThisInstruction(*inst);
                    if (debug) {
                        printLine(current_context_label);
                    }
                }
            }
            setForwardOut(current_pair.first, current_pair.second, prev);
//            CS_BB_OUT[current_pair].first=prev;
        } else//step 22
        {
            //step 23
            //NormalFlowFunction
            setForwardOut(current_pair.first, current_pair.second, NormalFlowFunctionForward(current_pair));
//            CS_BB_OUT[current_pair].first=NormalFlowFunctionForward(current_pair);
        }
        bool changed = false;
        if (!EqualDataFlowValuesForward(previous_value_at_out_of_this_node, getOut(current_pair.first,
                                                                                   current_pair.second).first)) // CS_BB_OUT[current_pair].first)
        {
            changed = true;
        }
        if (changed)//step 24
        {
            //not yet converged
            for (auto succ_bb:successors(bb))//step 25
            {
                //step 26
//                if (!forward_worklist_contains_this_entry[make_pair(current_context_label, succ_bb)]) {
//                    forward_worklist.push(make_pair(current_context_label, succ_bb));
//                    forward_worklist_contains_this_entry[make_pair(current_context_label, succ_bb)] = true;
//                }
                forward_worklist.workInsert({current_context_label,succ_bb});
                if (direction == "bidirectional") {
//                    if (!backward_worklist_contains_this_entry[make_pair(current_context_label, succ_bb)]) {
//                        backward_worklist.push(make_pair(current_context_label, succ_bb));
//                        backward_worklist_contains_this_entry[make_pair(current_context_label, succ_bb)] = true;
//                    }
                    backward_worklist.workInsert({current_context_label,succ_bb});
                }
            }

        }
        if (bb == &function.back())//step 27
        {
            //last node/exit node

            //step 28
            setForwardOutflowForThisContext(current_context_label, getPurelyGlobalComponentForward(
                    getOut(current_pair.first,
                           current_pair.second).first));//setting forward outflow CS_BB_OUT[current_pair].first
//            context_label_to_context_object_map[current_context_label].second.second.first=getPurelyGlobalComponentForward(CS_BB_OUT[current_pair].first);//setting forward outflow
            bool flag = false;
            for (auto context_inst_pairs:context_transition_graph)//step 29
            {
                if (context_inst_pairs.second == current_context_label)//matching the called function
                {
                    //step 30
                    BasicBlock *bb = context_inst_pairs.first.second->getParent();
                    pair<int, BasicBlock *> context_bb_pair = make_pair(context_inst_pairs.first.first, bb);
                    forward_worklist.workInsert(context_bb_pair);
//                    if (!forward_worklist_contains_this_entry[context_bb_pair]) {
//                        forward_worklist.push(context_bb_pair);
//                        forward_worklist_contains_this_entry[context_bb_pair] = true;
//                    }
                    if (direction == "bidirectional") {
                        backward_worklist.workInsert(context_bb_pair);
//                        if (!backward_worklist_contains_this_entry[context_bb_pair]) {
//                            backward_worklist.push(context_bb_pair);
//                            backward_worklist_contains_this_entry[context_bb_pair] = true;
//                        }
                    }
                }
                if (context_inst_pairs.first.first == current_context_label) {
                    flag = true;
                }
            }
            if (not flag) {
//                freeMemory(current_context_label);
            }
        }
    }
}

template<class F, class B>
F Analysis<F, B>::NormalFlowFunctionForward(pair<int, BasicBlock *> current_pair_of_context_label_and_bb) {
    BasicBlock &b = *(current_pair_of_context_label_and_bb.second);
    F prev = getIn(current_pair_of_context_label_and_bb.first,
                   current_pair_of_context_label_and_bb.second).first; // CS_BB_IN[current_pair_of_context_label_and_bb].first;
    //traverse a basic block in forward direction
    for (auto inst = &*(b.begin()); inst != nullptr; inst = inst->getNextNonDebugInstruction()) {
        if (debug) {
            printLine(current_pair_of_context_label_and_bb.first);
            llvm::outs() << *inst << "\n";
            llvm::outs() << "IN: ";
            printDataFlowValuesForward(prev);
        }
        setForwardComponentAtInOfThisInstruction(&(*inst), prev);//compute IN from previous OUT-value
        F new_prev = computeOutFromIn(*inst);
        setForwardComponentAtOutOfThisInstruction(&(*inst), new_prev);
        if (debug) {
            llvm::outs() << "OUT: ";
            printDataFlowValuesForward(new_prev);
        }
        prev = getForwardComponentAtOutOfThisInstruction(*inst);
        if (debug) {
            printLine(current_pair_of_context_label_and_bb.first);
        }
    }
    return prev;
}


template<class F, class B>
int Analysis<F, B>::check_if_context_already_exists(llvm::Function *function, const pair<F, B>& Inflow, const pair<F, B>& Outflow) {
    if (std::is_same<B, NoAnalysisType>::value) {
        //forward only
        for (auto set_itr:ProcedureContext) {
            Context<F, B> *current_object = context_label_to_context_object_map[set_itr];
            F new_context_values = Inflow.first;
            F current_context_values = current_object->getInflowValue().first;
            if (function->getName() == current_object->getFunction()->getName() &&
                EqualDataFlowValuesForward(new_context_values, current_context_values)) {
                if (debug) {
                    llvm::outs()
                            << "======================================================================================"
                            << "\n";
                    llvm::outs() << "Context found!!!!!" << "\n";
                    llvm::outs() << "LABEL: " << set_itr << "\n";
                    llvm::outs()
                            << "======================================================================================"
                            << "\n";
                }
                return set_itr;
            }
        }
    } else if (std::is_same<F, NoAnalysisType>::value) {
        // Backward Analysis
        for (auto set_itr:ProcedureContext) {
            Context<F, B> *current_object = context_label_to_context_object_map[set_itr];
            B new_context_values = Inflow.second;
            B current_context_values = current_object->getInflowValue().second;
            process_mem_usage(this->vm, this->rss);
            this->total_memory = max(this->total_memory, this->vm);
            if (function->getName() == current_object->getFunction()->getName() &&
                EqualDataFlowValuesBackward(new_context_values, current_context_values)) {
                if (debug) {
                    llvm::outs()
                            << "======================================================================================"
                            << "\n";
                    llvm::outs() << "Context found!!!!!" << "\n";
                    llvm::outs() << "LABEL: " << set_itr << "\n";
                    llvm::outs()
                            << "======================================================================================"
                            << "\n";
                }
                return set_itr;
            }
        }
    } else {
        // Todo for Bidirectional Analysis
        for(auto set_itr : ProcedureContext) {
            Context<F, B> *current_object = context_label_to_context_object_map[set_itr];
            F new_context_values_forward = Inflow.first;
            B new_context_values_backward = Inflow.second;
            F current_context_values_forward = current_object->getInflowValue().first;
            B current_context_values_backward = current_object->getInflowValue().second;
            if(function->getName() == current_object->getFunction()->getName() &&
            EqualDataFlowValuesForward(new_context_values_forward, current_context_values_forward) &&
            EqualDataFlowValuesBackward(new_context_values_backward, current_context_values_backward)) {
                if (debug) {
                    llvm::outs()
                            << "======================================================================================"
                            << "\n";
                    llvm::outs() << "Context found!!!!!" << "\n";
                    llvm::outs() << "LABEL: " << set_itr << "\n";
                    llvm::outs()
                            << "======================================================================================"
                            << "\n";
                }
                return set_itr;
            }
        }
    }
    return 0;
}

template<class F, class B>
void Analysis<F, B>::printWorklistMaps() {

}

template<class F, class B>
void Analysis<F, B>::doAnalysisBackward() {
    while (not backward_worklist.empty())//step 2
    {
        //step 3 and 4
        pair<int, BasicBlock *> current_pair = backward_worklist.workDelete();
        int current_context_label;
        BasicBlock *bb;
        current_context_label = current_pair.first;
        setProcessingContextLabel(current_context_label);
        bb = current_pair.second;

        BasicBlock &b = *bb;
        Function *f = context_label_to_context_object_map[current_context_label]->getFunction();
        Function &function = *f;

        //step 5
        if (bb != (&function.back())) {
            //step 6
            setBackwardOut(current_pair.first, current_pair.second, getInitialisationValueBackward());
//            CS_BB_OUT[current_pair].second=getInitialisationValueBackward();
            //step 7 and 8
            for (auto succ_bb:successors(bb)) {
                setBackwardOut(
                        current_pair.first,
                        current_pair.second,
                        performMeetBackward(
                                getOut(current_pair.first, current_pair.second).second,
                                getIn(current_pair.first, succ_bb).second
                        )
                );
                printDataFlowValuesBackward(getOut(current_pair.first, current_pair.second).second);
//                CS_BB_OUT[current_pair].second=performMeetBackward(CS_BB_OUT[current_pair].second,CS_BB_IN[make_pair(current_pair.first,succ_bb)].second);
            }
        } else {
            //Out value of this node is same as INFLOW value
            setBackwardOut(
                    current_pair.first,
                    current_pair.second,
                    getBackwardInflowForThisContext(current_context_label)
            );
//            CS_BB_OUT[current_pair].second=getBackwardInflowForThisContext(current_context_label);
            // CS_BB_OUT[current_pair].second=context_label_to_context_object_map[current_context_label].second.first.second;
        }

        //step 9
        F a1 = getIn(current_pair.first, current_pair.second).first; //CS_BB_IN[current_pair].first;
        B d1 = getIn(current_pair.first, current_pair.second).second; //CS_BB_OUT[current_pair].second;

        B previous_value_at_in_of_this_node = getIn(current_pair.first,
                                                    current_pair.second).second; //CS_BB_IN[current_pair].second;
        //step 10
        bool contains_a_method_call = false;
        for (auto inst = &*(b.begin()); inst != nullptr; inst = inst->getNextNonDebugInstruction()) {
            Instruction &I = *inst;
            if (CallInst *ci = dyn_cast<CallInst>(&I)) {
                Function *target_function = ci->getCalledFunction();
                if (not target_function || target_function->isDeclaration() || isAnIgnorableDebugInstruction(&I)) {
                    continue;//this is an inbuilt function so doesnt need to be processed.
                }
                // contains_a_method_call=true;
                contains_a_method_call = true;
                break;
            }
        }
        if (contains_a_method_call) {
            B prev = getOut(current_pair.first, current_pair.second).second; //CS_BB_OUT[current_pair].second;

            //step 11
            for (auto inst = b.rbegin(); inst != b.rend(); inst++) {
                Instruction &I = *inst;
                if (CallInst *ci = dyn_cast<CallInst>(&I)) {

                    Function *target_function = ci->getCalledFunction();
                    if (not target_function || target_function->isDeclaration() || isAnIgnorableDebugInstruction(&I)) {
                        continue;
                    }

                    /*
                    At the call instruction, the value at OUT should be splitted into two components:
                    1) Purely Global and 2) Mixed.
                    The purely global component is given to the end of callee.
                    */
                    //step 12
                    pair<F, B> inflow_pair = CallInflowFunction(current_context_label, target_function, bb, a1, prev);
                    F a2 = inflow_pair.first;
                    B d2 = inflow_pair.second;

                    // F new_inflow_forward;
                    // B new_inflow_backward=getPurelyGlobalComponentBackward(prev);//prev;
                    F new_outflow_forward;
                    B new_outflow_backward;



                    //step 13
                    // pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object=make_pair(target_function,make_pair(make_pair(new_inflow_forward,new_inflow_backward),make_pair(new_outflow_forward,new_outflow_backward)));
//                    pair<Function*,pair<pair<F,B>,pair<F,B>>> new_context_object=make_pair(target_function,make_pair(make_pair(a2,d2),make_pair(new_outflow_forward,new_outflow_backward)));

                    setBackwardComponentAtOutOfThisInstruction(&(*inst), prev);//compute OUT from previous IN-value

                    int matching_context_label = 0;
                    matching_context_label = check_if_context_already_exists(target_function, {a2, d2},
                                                                             {new_outflow_forward,
                                                                              new_outflow_backward});
                    if (matching_context_label > 0)//step 15
                    {
                        if (debug) {
                            printLine(current_context_label);
                            llvm::outs() << *inst << "\n";
                            llvm::outs() << "OUT: ";
                            printDataFlowValuesBackward(d1);
                        }
                        //step 14
                        pair<int, Instruction *> mypair = make_pair(current_context_label, &(*inst));
                        context_transition_graph[mypair] = matching_context_label;

                        //step 16 and 17
                        F a3 = getForwardOutflowForThisContext(matching_context_label);
                        B d3 = getBackwardOutflowForThisContext(matching_context_label);

                        pair<F, B> outflow_pair = CallOutflowFunction(current_context_label, target_function, bb, a3,
                                                                      d3, a1, d1);
                        F a4 = outflow_pair.first;
                        B value_to_be_meet_with_prev_in = outflow_pair.second;

                        //step 18 and 19

                        /*
                        At the call instruction, the value at OUT should be splitted into two components.
                        The purely global component is given to the callee while the mixed component is propagated
                        to IN of this instruction after executing computeINfromOUT() on it.
                        */

                        /*
                        At the IN of this instruction, the value from START of callee procedure is to be merged
                        with the local(mixed) value propagated from OUT. Note that this merging "isn't"
                        exactly (necessarily) the meet between these two values.
                        */

                        // B merged_value_at_in_of_instruction=getCombinedValuesAtCallBackward(new_outflow_backward,value_to_be_propagated_to_in_of_instruction);//combining d4 and d5


                        /*
                        As explained in ip-vasco,pdf, we need to perform meet with the original value of IN
                        of this instruction to avoid the oscillation problem.
                        */

                        setBackwardComponentAtInOfThisInstruction(&(*inst),
                                                                  performMeetBackward(value_to_be_meet_with_prev_in,
                                                                                      getBackwardComponentAtInOfThisInstruction(
                                                                                              (*inst))));

                        prev = getBackwardComponentAtInOfThisInstruction((*inst));
                        if (debug) {
                            llvm::outs() << "IN: ";
                            printDataFlowValuesBackward(prev);
                            printLine(current_context_label);
                        }

                    } else//step 20
                    {
                        //creating a new context
                        INIT_CONTEXT(target_function, {a2, d2}, {new_outflow_forward, new_outflow_backward});//step 21

                        pair<int, Instruction *> mypair = make_pair(current_context_label, &(*inst));
                        //step 14
                        context_transition_graph[mypair] = context_label_counter;
                        return;
                    }
                } else {
                    if (debug) {
                        printLine(current_context_label);
                        llvm::outs() << *inst << "\n";
                        llvm::outs() << "OUT: ";
                        printDataFlowValuesBackward(prev);
                    }
                    setBackwardComponentAtOutOfThisInstruction(&(*inst), prev);//compute OUT from previous IN-value
                    setBackwardComponentAtInOfThisInstruction(&(*inst), computeInFromOut(*inst));
                    prev = getBackwardComponentAtInOfThisInstruction((*inst));
                    if (debug) {
                        llvm::outs() << "IN: ";
                        printDataFlowValuesBackward(prev);
                        printLine(current_context_label);
                    }
                }
            }
            setBackwardIn(current_pair.first, current_pair.second, prev);
//            CS_BB_IN[current_pair].second=prev;
        } else//step 22
        {
            //step 23
            //NormalFlowFunction
            setBackwardIn(current_pair.first, current_pair.second, NormalFlowFunctionBackward(current_pair));
//            CS_BB_IN[current_pair].second=NormalFlowFunctionBackward(current_pair);

        }
        bool changed = false;
        if (!EqualDataFlowValuesBackward(previous_value_at_in_of_this_node,
                                         getIn(current_pair.first, current_pair.second).second)) {
            changed = true;
        }
        if (changed)//step 24
        {
            //not yet converged
            for (auto pred_bb:predecessors(bb))//step 25
            {
                //step 26
                if (direction == "bidirectional") {
                    forward_worklist.workInsert(make_pair(current_context_label, pred_bb));
                }
                backward_worklist.workInsert(make_pair(current_context_label, pred_bb));
            }

        }

        if (bb == &function.getEntryBlock())//step 27
        {

            //step 28
            setBackwardOutflowForThisContext(current_context_label, getPurelyGlobalComponentBackward(
                    getIn(current_pair.first, current_pair.second).second));//setting backward outflow

            for (auto context_inst_pairs:context_transition_graph)//step 29
            {
                if (context_inst_pairs.second == current_context_label)//matching the called function
                {
                    //step 30

                    BasicBlock *bb = context_inst_pairs.first.second->getParent();
                    pair<int, BasicBlock *> context_bb_pair = make_pair(context_inst_pairs.first.first, bb);
                    if (direction == "bidirectional") {
                        forward_worklist.workInsert(context_bb_pair);
                    }
                    backward_worklist.workInsert(context_bb_pair);
                }
            }
        }
        process_mem_usage(this->vm, this->rss);
        this->total_memory = max(this->total_memory, this->vm);
    }
}

template<class F, class B>
B Analysis<F, B>::NormalFlowFunctionBackward(pair<int, BasicBlock *> current_pair_of_context_label_and_bb) {
    BasicBlock &b = *(current_pair_of_context_label_and_bb.second);
    B prev = getOut(current_pair_of_context_label_and_bb.first,
                    current_pair_of_context_label_and_bb.second).second; //CS_BB_OUT[current_pair_of_context_label_and_bb].second;
    F prev_f = getOut(current_pair_of_context_label_and_bb.first,
                      current_pair_of_context_label_and_bb.second).first; //CS_BB_OUT[current_pair_of_context_label_and_bb].first;
    //traverse a basic block in backward direction
    for (auto inst = &*(b.rbegin()); inst != nullptr; inst = inst->getPrevNonDebugInstruction()) {
        if (debug) {
            printLine(current_pair_of_context_label_and_bb.first);
            llvm::outs() << *inst << "\n";
            llvm::outs() << "OUT: ";
            printDataFlowValuesBackward(prev);
        }
        setBackwardComponentAtOutOfThisInstruction(&(*inst), prev);//compute OUT from previous IN-value
        B new_dfv = computeInFromOut(*inst);
        setBackwardComponentAtInOfThisInstruction(&(*inst), new_dfv);
        if (debug) {
            llvm::outs() << "IN: ";
            printDataFlowValuesBackward(new_dfv);
            printLine(current_pair_of_context_label_and_bb.first);
        }
        prev = getBackwardComponentAtInOfThisInstruction(*inst);
    }
    return prev;
}


template<class F, class B>
void Analysis<F, B>::performSplittingBB(Function &function) {
    int flag = 0;
    Instruction *I = nullptr;
    bool previousInstructionWasSplitted = false;
    bool previousInstructionWasCallInstruction = false;
    map<Instruction *, bool> isSplittedOnThisInstruction;
    for (BasicBlock *BB:inverse_post_order(&function.back())) {
        BasicBlock &b = *BB;
        previousInstructionWasSplitted = true;
        previousInstructionWasCallInstruction = false;
        for (auto inst = &*(b.begin()); inst != nullptr; inst = inst->getNextNonDebugInstruction()) {
            I = &(*inst);
            if (inst == &*(b.begin())) {
                if (isa<CallInst>(*I)) {
                    CallInst *ci = dyn_cast<CallInst>(I);

                    Function *target_function = ci->getCalledFunction();
                    if (not target_function || target_function->isDeclaration() || isAnIgnorableDebugInstruction(I)) {
                        continue;//this is an inbuilt function so doesn't need to be processed.
                    }
                    isSplittedOnThisInstruction[I] = false;
                    previousInstructionWasCallInstruction = true;
                    previousInstructionWasSplitted = true;//creating a false positive
                }
                continue;
            }

            if (isa<BranchInst>(*I)) {

            } else if (isa<CallInst>(*I)) {
                CallInst *ci = dyn_cast<CallInst>(I);


                Function *target_function = ci->getCalledFunction();
                if (not target_function || target_function->isDeclaration() || isAnIgnorableDebugInstruction(I)) {
                    continue;
                }
                isSplittedOnThisInstruction[I] = true;
                previousInstructionWasCallInstruction = true;
                previousInstructionWasSplitted = true;
            } else {
                if (previousInstructionWasSplitted) {
                    if (previousInstructionWasCallInstruction) {
                        isSplittedOnThisInstruction[I] = true;
                        previousInstructionWasSplitted = true;
                    } else {
                        previousInstructionWasSplitted = false;
                    }
                } else {
                    //do nothing
                }
                previousInstructionWasCallInstruction = false;
            }
        }
    }
    BasicBlock *containingBB;

    for (auto split_here:isSplittedOnThisInstruction) {
        if (split_here.second == false)//no splitting to be done
            continue;
        containingBB = split_here.first->getParent();
        containingBB->splitBasicBlock(split_here.first);
    }
    // function.viewCFG();
}

template<class F, class B>
void Analysis<F, B>::printContext() {
    llvm::outs() << "\n";
    for (auto label : ProcedureContext) {
        llvm::outs()
                << "=================================================================================================="
                << "\n";
        auto context = context_label_to_context_object_map[label];
        llvm::outs() << "LABEL: " << label << "\n";
        llvm::outs() << "FUNCTION NAME: " << context->getFunction()->getName() << "\n";
        llvm::outs() << "INFLOW VALUE: ";
        printDataFlowValuesForward(context->getInflowValue().first);
        printDataFlowValuesBackward(context->getInflowValue().second);
        llvm::outs() << "OUTFLOW VALUE: ";
        printDataFlowValuesForward(context->getOutflowValue().first);
        printDataFlowValuesBackward(context->getOutflowValue().second);
        llvm::outs() << "\n";
        llvm::outs()
                << "=================================================================================================="
                << "\n";
    }
}


#endif

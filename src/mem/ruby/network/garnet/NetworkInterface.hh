/*
 * Copyright (c) 2020 Advanced Micro Devices, Inc.
 * Copyright (c) 2020 Inria
 * Copyright (c) 2016 Georgia Institute of Technology
 * Copyright (c) 2008 Princeton University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef __MEM_RUBY_NETWORK_GARNET_0_NETWORKINTERFACE_HH__
#define __MEM_RUBY_NETWORK_GARNET_0_NETWORKINTERFACE_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/garnet/CommonTypes.hh"
#include "mem/ruby/network/garnet/Credit.hh"
#include "mem/ruby/network/garnet/CreditLink.hh"
#include "mem/ruby/network/garnet/GarnetNetwork.hh"
#include "mem/ruby/network/garnet/NetworkLink.hh"
#include "mem/ruby/network/garnet/OutVcState.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "params/GarnetNetworkInterface.hh"


//yzkth
#include "sim/eventq.hh" // 包含 Event 相关头文件
#include <filesystem>   // C++17 std::filesystem
#include <thread>       // std::this_thread
#include "sim/simulate.hh" // 包含 simulate_limit_event"" 
#include <unistd.h>




#include "debug/yzzzzNI.hh"


#include <onnxruntime_cxx_api.h>

#define yz250203LeakyBucketOn 
//#define yz250218RLReadFile
#define yzRecordActualInjRate
#define yzBufferLoadADInjRate250224
namespace gem5
{

namespace ruby
{

class MessageBuffer;

namespace garnet
{
  
    
class flitBuffer;

class NetworkInterface : public ClockedObject, public Consumer
{
  public:
    typedef GarnetNetworkInterfaceParams Params;
    NetworkInterface(const Params &p);
    ~NetworkInterface() = default;
    //yzkth
    int yz_ADtokenGenerated = 0;
    int yz_ADtokenUsed = 0;
    int yz_ADtokenWasted = 0;
    double yz_tokenInBucket = 0;
    float yz_InjRate = 1.0999;
    EventFunctionWrapper m_yztick_event,m_yzRecordSelfInjPacket; // 添加事件成员
    void yzperTickFunction();           // 添加 perTickFunction() 声明
    void yzOneNI_recordOnePacket(int  sourceNIID, int dest_niID,int recvNIID  , int onWhichVNet,float in_queueing_delay,  float in_network_delay) ;
    
   


    int yzPeriodLastActualInjPacketCount = 0;
    float yzLastPacketPeriodAvgQueueDelay = 0;
    float yzLastPacketPeriodAvgNetDelay = 0;
    int   yzLastPacketPeriodCount = 0;

    
    

    int yzPeriodActualInjPacketCount = 0;
    float yzPacketPeriodAvgQueueDelay = 0;
    float yzPacketPeriodAvgNetDelay = 0;
    int   yzPacketPeriodCount = 0;
   



    float yzPacketLastPeriodSumNetDelay = 0;
    float yzPacketLastPeriodSumQueueDelay = 0;
    float yzPacketPeriodSumQueueDelay = 0;
    float yzPacketPeriodSumNetDelay = 0;
    int   yzPacketLastThreshold = 0;
    int tempThreshold = 0;


    float yzActionFromPython = 0.5;
   
    

    static  int totalWrittenNIs  ; // 记录已经写入的 NI 数量
    Tick pythonReadTick = 0;
 

    
    void  m_yzRecordSelfInjPacketFunction();

    const int yzResetTokenPeriod = 20000; // 重置 Token 的周期, clock cycle rather than ticks
    void yzWritePeriodStateFile(int in_m_id);
    void yzReadAndStuckForPythonFIle(int in_m_id);
    long long yzLastPeriodCycleForTokenGen = 0;
     int yz_ADNewPeriodtokenGenerated;
    void  yzResetBucketPeriod();
    int yzCheckIniEvent = 0;
    int yzLast_ADNewPeriodtokenGenerated ;

    static float yz_shareActionAllNIs;
    
   static int  yzstate0_lastInj  ;
   static  float yzstate1_lastRec ;
   static  float yzstate2_lastQueudelay  ;
   static int    yzstate3_lastNetDelay  ;
   static int  yzstate4_curInj  ;
   static float yzstate5_curRec  ;
   static  float yzstate6_curQueudelay  ;
   static  int   yzstate7_curNetDelay  ;




    static float  yz_shareNICPURequestList[128]; // 64 个 NI 的 CPU 请求列表
    int yz_preCPUInjSignalCount = 0;
    int yz_curCPUInjSignalCount = 0;

    bool newBashEnable = true;
    static float yz_shareInjRateNoC;
    static float yz_shareNoCTotalPacketCount;

    

    int thisNodeControledLastPeriod = 0;



    



   bool yzTestOnnxModel();
   bool readRLModelInferenceTest( ) ;

    void addInPort(NetworkLink *in_link, CreditLink *credit_link);
    void addOutPort(NetworkLink *out_link, CreditLink *credit_link,
        SwitchID router_id, uint32_t consumerVcs);

    void dequeueCallback();
    void wakeup();
    void addNode(std::vector<MessageBuffer *> &inNode,
                 std::vector<MessageBuffer *> &outNode);

    void print(std::ostream& out) const;
    int get_vnet(int vc);
    void init_net_ptr(GarnetNetwork *net_ptr) { m_net_ptr = net_ptr; }

    bool functionalRead(Packet *pkt, WriteMask &mask);
    uint32_t functionalWrite(Packet *);

    void scheduleFlit(flit *t_flit);

    int get_router_id(int vnet)
    {
        OutputPort *oPort = getOutportForVnet(vnet);
        assert(oPort);
        return oPort->routerID();
    }

    class OutputPort
    {
      public:
          OutputPort(NetworkLink *outLink, CreditLink *creditLink,
              int routerID)
          {
              _vnets = outLink->mVnets;
              _outFlitQueue = new flitBuffer();

              _outNetLink = outLink;
              _inCreditLink = creditLink;

              _routerID = routerID;
              _bitWidth = outLink->bitWidth;
              _vcRoundRobin = 0;

          }

          flitBuffer *
          outFlitQueue()
          {
              return _outFlitQueue;
          }

          NetworkLink *
          outNetLink()
          {
              return _outNetLink;
          }

          CreditLink *
          inCreditLink()
          {
              return _inCreditLink;
          }

          int
          routerID()
          {
              return _routerID;
          }

          uint32_t bitWidth()
          {
              return _bitWidth;
          }

          bool isVnetSupported(int pVnet)
          {
              if (!_vnets.size()) {
                  return true;
              }

              for (auto &it : _vnets) {
                  if (it == pVnet) {
                      return true;
                  }
              }
              return false;

          }

          std::string
          printVnets()
          {
              std::stringstream ss;
              for (auto &it : _vnets) {
                  ss << it;
                  ss << " ";
              }
              return ss.str();
          }

          int vcRoundRobin()
          {
              return _vcRoundRobin;
          }

          void vcRoundRobin(int vc)
          {
              _vcRoundRobin = vc;
          }


      private:
          std::vector<int> _vnets;
          flitBuffer *_outFlitQueue;

          NetworkLink *_outNetLink;
          CreditLink *_inCreditLink;

          int _vcRoundRobin; // For round robin scheduling

          int _routerID;
          uint32_t _bitWidth;
    };

    class InputPort
    {
      public:
          InputPort(NetworkLink *inLink, CreditLink *creditLink)
          {
              _vnets = inLink->mVnets;
              _outCreditQueue = new flitBuffer();

              _inNetLink = inLink;
              _outCreditLink = creditLink;
              _bitWidth = inLink->bitWidth;
          }

          flitBuffer *
          outCreditQueue()
          {
              return _outCreditQueue;
          }

          NetworkLink *
          inNetLink()
          {
              return _inNetLink;
          }

          CreditLink *
          outCreditLink()
          {
              return _outCreditLink;
          }

          bool isVnetSupported(int pVnet)
          {
              if (!_vnets.size()) {
                  return true;
              }

              for (auto &it : _vnets) {
                  if (it == pVnet) {
                      return true;
                  }
              }
              return false;

          }

          void sendCredit(Credit *cFlit)
          {
              _outCreditQueue->insert(cFlit);
          }

          uint32_t bitWidth()
          {
              return _bitWidth;
          }

          std::string
          printVnets()
          {
              std::stringstream ss;
              for (auto &it : _vnets) {
                  ss << it;
                  ss << " ";
              }
              return ss.str();
          }

          // Queue for stalled flits
          std::deque<flit *> m_stall_queue;
          bool messageEnqueuedThisCycle;
      private:
          std::vector<int> _vnets;
          flitBuffer *_outCreditQueue;

          NetworkLink *_inNetLink;
          CreditLink *_outCreditLink;
          uint32_t _bitWidth;
    };


  private:
    GarnetNetwork *m_net_ptr;
    const NodeID m_id;
    const int m_virtual_networks;
    int m_vc_per_vnet;
    std::vector<int> m_vc_allocator;
    std::vector<OutputPort *> outPorts;
    std::vector<InputPort *> inPorts;
    int m_deadlock_threshold;
    std::vector<OutVcState> outVcState;

    std::vector<int> m_stall_count;

    // Input Flit Buffers
    // The flit buffers which will serve the Consumer
    std::vector<flitBuffer>  niOutVcs;
    std::vector<Tick> m_ni_out_vcs_enqueue_time;

    // The Message buffers that takes messages from the protocol
    std::vector<MessageBuffer *> inNode_ptr;
    // The Message buffers that provides messages to the protocol
    std::vector<MessageBuffer *> outNode_ptr;
    // When a vc stays busy for a long time, it indicates a deadlock
    std::vector<int> vc_busy_counter;

    void checkStallQueue();
    bool flitisizeMessage(MsgPtr msg_ptr, int vnet);
    int calculateVC(int vnet);


    void scheduleOutputPort(OutputPort *oPort);
    void scheduleOutputLink();
    void checkReschedule();

    void incrementStats(flit *t_flit);

    InputPort *getInportForVnet(int vnet);
    OutputPort *getOutportForVnet(int vnet);

     bool yzModifiedflitisizeMessage(MsgPtr msg_ptr, int vnet);
};

} // namespace garnet
} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_NETWORK_GARNET_0_NETWORKINTERFACE_HH__

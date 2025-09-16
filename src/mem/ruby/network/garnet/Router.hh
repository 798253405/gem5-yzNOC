/*
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


#ifndef __MEM_RUBY_NETWORK_GARNET_0_ROUTER_HH__
#define __MEM_RUBY_NETWORK_GARNET_0_ROUTER_HH__

#include <iostream>
#include <memory>
#include <vector>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/BasicRouter.hh"
#include "mem/ruby/network/garnet/CommonTypes.hh"
#include "mem/ruby/network/garnet/CrossbarSwitch.hh"
#include "mem/ruby/network/garnet/GarnetNetwork.hh"
#include "mem/ruby/network/garnet/RoutingUnit.hh"
#include "mem/ruby/network/garnet/SwitchAllocator.hh"
#include "mem/ruby/network/garnet/flit.hh"
#include "params/GarnetRouter.hh"
#include "sim/dvfs_handler.hh"

namespace gem5
{

namespace ruby
{

class FaultModel;

namespace garnet
{

class NetworkLink;
class CreditLink;
class InputUnit;
class OutputUnit;

class Router : public BasicRouter, public Consumer
{
  public:
    typedef GarnetRouterParams Params;
    Router(const Params &p);

    ~Router() = default;

    void wakeup();
    void print(std::ostream& out) const {};

    void init();
    void addInPort(PortDirection inport_dirn, NetworkLink *link,
                   CreditLink *credit_link);
    void addOutPort(PortDirection outport_dirn, NetworkLink *link,
                    std::vector<NetDest>& routing_table_entry,
                    int link_weight, CreditLink *credit_link,
                    uint32_t consumerVcs);

    Cycles get_pipe_stages(){ return getEffectiveLatency(); }
    uint32_t get_num_vcs()       { return m_num_vcs; }
    uint32_t get_num_vnets()     { return m_virtual_networks; }
    uint32_t get_vc_per_vnet()   { return m_vc_per_vnet; }
    int get_num_inports()   { return m_input_unit.size(); }
    int get_num_outports()  { return m_output_unit.size(); }
    int get_id()            { return m_id; }

    void init_net_ptr(GarnetNetwork* net_ptr)
    {
        m_network_ptr = net_ptr;
    }

    GarnetNetwork* get_net_ptr()                    { return m_network_ptr; }

    InputUnit*
    getInputUnit(unsigned port)
    {
        assert(port < m_input_unit.size());
        return m_input_unit[port].get();
    }

    OutputUnit*
    getOutputUnit(unsigned port)
    {
        assert(port < m_output_unit.size());
        return m_output_unit[port].get();
    }

    int getBitWidth() { return m_bit_width; }

    PortDirection getOutportDirection(int outport);
    PortDirection getInportDirection(int inport);

    int route_compute(RouteInfo route, int inport, PortDirection direction);
    void grant_switch(int inport, flit *t_flit);
    void schedule_wakeup(Cycles time);

    std::string getPortDirectionName(PortDirection direction);
    void printFaultVector(std::ostream& out);
    void printAggregateFaultProbability(std::ostream& out);

    void regStats();
    void collateStats();
    void resetStats();

    // For Fault Model:
    bool get_fault_vector(int temperature, float fault_vector[]) {
        return m_network_ptr->fault_model->fault_vector(m_id, temperature,
                                                        fault_vector);
    }
    bool get_aggregate_fault_probability(int temperature,
                                         float *aggregate_fault_prob) {
        return m_network_ptr->fault_model->fault_prob(m_id, temperature,
                                                      aggregate_fault_prob);
    }

    bool functionalRead(Packet *pkt, WriteMask &mask);
    uint32_t functionalWrite(Packet *);

    // DVFS related functions
    enum DVFSLevel {
        DVFS_LOW = 0,    // Low voltage/frequency for power saving
        DVFS_MEDIUM = 1, // Default/medium voltage/frequency
        DVFS_HIGH = 2    // High voltage/frequency for high performance
    };
    
    void setDVFSLevel(DVFSLevel level);
    DVFSLevel getCurrentDVFSLevel() const { return m_current_dvfs_level; }
    void triggerDVFSChange(DVFSLevel level); // Interface for future network-aware DVFS
    void periodicDVFSUpdate(); // Periodic DVFS switching for testing
    
    // Override ClockedObject hook for clock period updates
    void clockPeriodUpdated() override;
    
    // Get effective latency with DVFS scaling applied
    Cycles getEffectiveLatency() const;

  private:
    Cycles m_latency;
    uint32_t m_virtual_networks, m_vc_per_vnet, m_num_vcs;
    uint32_t m_bit_width;
    GarnetNetwork *m_network_ptr;

    RoutingUnit routingUnit;
    SwitchAllocator switchAllocator;
    CrossbarSwitch crossbarSwitch;

    std::vector<std::shared_ptr<InputUnit>> m_input_unit;
    std::vector<std::shared_ptr<OutputUnit>> m_output_unit;

    // Statistical variables required for power computations
    statistics::Scalar m_buffer_reads;
    statistics::Scalar m_buffer_writes;

    statistics::Scalar m_sw_input_arbiter_activity;
    statistics::Scalar m_sw_output_arbiter_activity;

    statistics::Scalar m_crossbar_activity;
    
    // DVFS statistics
    statistics::Scalar m_dvfs_freq_mhz;
    statistics::Scalar m_dvfs_scale_factor;
    statistics::Scalar m_dvfs_effective_latency;

    // DVFS related variables
    DVFSLevel m_current_dvfs_level;
    EventFunctionWrapper m_dvfs_update_event;
    Tick m_dvfs_switch_interval; // Interval for periodic DVFS switching
    uint32_t m_dvfs_cycle_counter; // Counter for cycling through DVFS levels
    bool m_dvfs_enable_periodic; // Enable periodic DVFS switching
    double m_dvfs_freq[3]; // Frequency levels for LOW/MEDIUM/HIGH
    double m_dvfs_voltage[3]; // Voltage levels for LOW/MEDIUM/HIGH
    double m_freq_scale_factor; // Current frequency scaling factor (1.0 = medium freq)
    std::string m_dvfs_mode; // DVFS mode: low, medium, high, or cycle
};

} // namespace garnet
} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_NETWORK_GARNET_0_ROUTER_HH__

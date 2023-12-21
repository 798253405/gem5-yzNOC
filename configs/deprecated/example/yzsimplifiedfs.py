# Copyright (c) 2010-2013, 2016, 2019-2020 ARM Limited
# Copyright (c) 2020 Barkhausen Institut
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2012-2014 Mark D. Hill and David A. Wood
# Copyright (c) 2009-2011 Advanced Micro Devices, Inc.
# Copyright (c) 2006-2007 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import argparse
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal, warn
from m5.util.fdthelper import *
from gem5.isas import ISA
from gem5.runtime import get_runtime_isa

addToPath("../../")

from ruby import Ruby

from common.FSConfig import *
from common.SysPaths import *
from common.Benchmarks import *
from common import Simulation
from common import CacheConfig
from common import CpuConfig
from common import MemConfig
from common import ObjectList
from common.Caches import *
from common import Options


def cmd_line_template():
    if args.command_line and args.command_line_file:
        print(
            "Error: --command-line and --command-line-file are "
            "mutually exclusive"
        )
        sys.exit(1)
    if args.command_line:
        return args.command_line
    if args.command_line_file:
        return open(args.command_line_file).read().strip()
    return None


def build_test_system(np):
    cmdline = cmd_line_template()
    isa = get_runtime_isa()
    if isa == ISA.MIPS:
        test_sys = makeLinuxMipsSystem(test_mem_mode, bm[0], cmdline=cmdline)
   
    elif isa == ISA.X86:
        test_sys = makeLinuxX86System(
            test_mem_mode, np, bm[0], args.ruby, cmdline=cmdline
        )
    else:
        fatal("Incapable of building %s full system!", isa.name)

    # Set the cache line size for the entire system
    test_sys.cache_line_size = args.cacheline_size

    # Create a top-level voltage domain
    test_sys.voltage_domain = VoltageDomain(voltage=args.sys_voltage)

    # Create a source clock for the system and set the clock period
    test_sys.clk_domain = SrcClockDomain(
        clock=args.sys_clock, voltage_domain=test_sys.voltage_domain
    )

    # Create a CPU voltage domain
    test_sys.cpu_voltage_domain = VoltageDomain()

    # Create a source clock for the CPUs and set the clock period
    test_sys.cpu_clk_domain = SrcClockDomain(
        clock=args.cpu_clock, voltage_domain=test_sys.cpu_voltage_domain
    )

    if buildEnv["USE_RISCV_ISA"]:
        test_sys.workload.bootloader = args.kernel
    elif args.kernel is not None:
        print(f" yzzzz line 118 test_sys.workload.object_file = binary(args.kernel) ")
        test_sys.workload.object_file = binary(args.kernel)

    if args.script is not None:
        test_sys.readfile = args.script

    test_sys.init_param = args.init_param

    # For now, assign all the CPUs to the same clock domain
    test_sys.cpu = [
        TestCPUClass(clk_domain=test_sys.cpu_clk_domain, cpu_id=i)
        for i in range(np)
    ]

    if args.ruby:
        bootmem = getattr(test_sys, "_bootmem", None)
        Ruby.create_system(
            args, True, test_sys, test_sys.iobus, test_sys._dma_ports, bootmem
        )

        # Create a seperate clock domain for Ruby
        test_sys.ruby.clk_domain = SrcClockDomain(
            clock=args.ruby_clock, voltage_domain=test_sys.voltage_domain
        )

        # Connect the ruby io port to the PIO bus,
        # assuming that there is just one such port.
        test_sys.iobus.mem_side_ports = test_sys.ruby._io_port.in_ports

        for (i, cpu) in enumerate(test_sys.cpu):
            #
            # Tie the cpu ports to the correct ruby system ports
            #
            cpu.clk_domain = test_sys.cpu_clk_domain
            cpu.createThreads()
            cpu.createInterruptController()

            test_sys.ruby._cpu_ports[i].connectCpuPorts(cpu)

    else:
         print(" line 157 should not happen")

    if ObjectList.is_kvm_cpu(TestCPUClass) or ObjectList.is_kvm_cpu(
        FutureClass
    ):
        # Assign KVM CPUs to their own event queues / threads. This
        # has to be done after creating caches and other child objects
        # since these mustn't inherit the CPU event queue.
        for i, cpu in enumerate(test_sys.cpu):
            # Child objects usually inherit the parent's event
            # queue. Override that and use the same event queue for
            # all devices.
            for obj in cpu.descendants():
                obj.eventq_index = 0
            cpu.eventq_index = i + 1
        test_sys.kvm_vm = KvmVM()

    return test_sys

 


warn(
    "The fs.py script is deprecated. It will be removed in future releases of "
    " gem5."
)

# Add args
parser = argparse.ArgumentParser()
Options.addCommonOptions(parser)
Options.addFSOptions(parser)

# Add the ruby specific and protocol specific args
if "--ruby" in sys.argv:
    Ruby.define_options(parser)

args = parser.parse_args()

# system under test can be any CPU
(TestCPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(args)

# Match the memories with the CPUs, based on the options for the test system
TestMemClass = Simulation.setMemClass(args)

if args.benchmark:   
        print(f" yzzzz 203 args.benchmark should not happen")
        print(f"Error benchmark {args.benchmark} has not been defined.")
        print(f"Valid benchmarks are: {DefinedBenchmarks}")

else:
    if args.dual:
        print(f" yzzzz 209 args.dualshould not happen")         
    else:
        bm = [
            SysConfig(
                disks=args.disk_image,
                rootdev=args.root_device,
                mem=args.mem_size,
                os_type=args.os_type,
            )
        ]
        print(f" yzzz line 219 bm[0] is", bm[0])    
np = args.num_cpus

test_sys = build_test_system(np)

if len(bm) == 2:
    print(f" yzzzz  225 len(bm) == 2 should not happen")    
elif len(bm) == 1 and args.dist:
    # This system is part of a dist-gem5 simulation
    print(f" yzzzz 228 len(bm) == 1 and args.dist  should not happen")   
elif len(bm) == 1:
    print("yzzz 230 print we are using  root = Root(full_system=True, system=test_sys) \n")
    root = Root(full_system=True, system=test_sys)
else:
    print("Error I don't know how to create more than 2 systems.")
    sys.exit(1)

if ObjectList.is_kvm_cpu(TestCPUClass) or ObjectList.is_kvm_cpu(FutureClass):
    # Required for running kvm on multiple host cores.
    # Uses gem5's parallel event queue feature
    # Note: The simulator is quite picky about this number!
    root.sim_quantum = int(1e9)  # 1 ms

if args.timesync:
    root.time_sync_enable = True

if args.frame_capture:
    VncServer.frame_capture = True
 

if args.wait_gdb:
    test_sys.workload.wait_for_remote_gdb = True

Simulation.setWorkCountOptions(test_sys, args)
Simulation.run(args, root, test_sys, FutureClass)

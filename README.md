# falco

Falco is an out-of-order superscalar developped at Embedded Intelligent Systems Lab at NYCU (a.k.a. NCTU). The current release is just a 0.1 preview version. The superscalar can be synthesized for Xilinx FPGA development platform KC705 at 75Mhz. Currently, we provide two different ROMs in the processor cores to demonstrate the execution of Dhrystone and CoreMark, with the outputs sent to the UART device.

Falco is a 32-bit RISC-V core supporting RV32-IM instruction set. It has a 8-stage integer and 10-stage load/store pipeline. It does speculative execution with three-issue structure (two integer instructions and one load/store instruction). The microarchitecture is as follows:

![](docs/falco.jpg)

Currentyl Falco performance is around 1.84 DMIPS/Mhz and 2.67 CoreMark/Mhz.

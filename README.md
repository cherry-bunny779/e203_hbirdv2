e203_hbirdv2 RVFI Wrapper
===============================

About
-----

This repository hosts the project to make an RISC-V Formal Verification interface for the Hummingbirdv2 E203 Core and SoC opensourced by [Nuclei System Technology](www.nucleisys.com). You can find the parent repository [here](https://github.com/riscv-mcu/e203_hbirdv2).

Verification Procedure
-----

1. Set up RISC-V Formal Verification Framework (RVFI)
    
    Follow this [link](https://github.com/YosysHQ/riscv-formal) to Yosys' RVFI homepage and set up the verification environment.

2. Prepare the files

    Navigate to the following directory

    ```
    cd riscv-formal/cores/
    ```

    and clone this github repository.

3. Generate Checks

    Navigate to the following directory
    ```
    cd e203_hbirdv2
    ```

    and run the following command to generate the RVFI checks
    ```
    python3 ../../checks/genchecks.py
    ```

4. Pre-process the generated tests

    For reasons yet obscure [To-Do], the "depth" and "skip" fields in the .sby files need to be corrected for the generated tests. They are off by +1. As a temporary solution, copy and run the preprocessing scipt. Make sure you are in the directory /riscv-formal/cores/e203_hbird
    ```
    cp process_sby.py ./checks
    python3 process_sby.py
    ```

5. Compile and Run RVFI Tests

    Navigate to /riscv-formal/cores/e203_hbird
    ```
    cd ..
    ```

    and use the following commands to run RVFI Tests
    All Tests:
    ```
    make -C checks -j$(nproc)
    ```

    A specific Test:
    ```
    sby -f ./checks/<name_of_test>.sby
    ```

Known Issues [To-Do]
-----

1. Incorrect "depth" and "skip" fields, see step 4 in Verification Procedure

    This may be caused by how the **_rvfi_valid_** signal is generated in the wrapper structure. Note that the 'depth' field is most relevent since 'skip' effectively only speeds up the verification process.

2. Unable to run "unique" and "liveness" tests

    For the purpose of the project, this is fine as we only need to ensure all of the instruction and pc test results are generated.

3. Redundant RTL Data

   To simplify manual signal routing though down->top submodules that concieve the RVFI interface, relevant submodules have been merged into a single file: e203_hbirdv2.v. Could use some clean-up.

4. Control and Status Registers (CSRs) not monitored

   RVFI supports generating CSRs tests, but these tests are skipped for the verification of this core 

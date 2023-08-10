# Bluetooth&reg; LE CTS client

This code example demonstrates the implementation of a simple AIROC&trade; Bluetooth&reg; LE GAP Peripheral - GATT Client with Current Time Service ([CTS](https://www.bluetooth.com/xml-viewer/?src=https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Services/org.bluetooth.service.current_time.xml)) using PSoC&trade;  6 MCU, AIROC&trade;  CYW20829, and ModusToolbox&trade;  software environment.

In this code example, the kit advertises as 'CTS Client' and after connection with a CTS-based server, enables notifications for the 'Current Time' characteristic. The time and date received through the notification are printed on the serial terminal.

This code example along with Bluetooth&reg; LE CTS Server CE are low-power enabled for Bluetooth&reg; LE and can be used to measure the current consumption by PSoC&trade; 6 (not optimized for power) and CYW43xxx when using AIROC&trade; Bluetooth&reg; LE feature. See [AN227910: Low-power system design with CYW43012 and PSoC&trade; 6 MCU](https://www.infineon.com/dgdl/Infineon-AN227910_Low-power_system_design_with_AIROC_CYW43012_Wi-Fi_%26_Bluetooth_combo_chip_and_PSoC_6_MCU-ApplicationNotes-v03_00-EN.pdf?fileId=8ac78c8c7cdc391c017d0d39b66166f3) to learn about Bluetooth&reg; power optimization techniques and power measurement using this CE.

Use this code example with [Bluetooth&reg; LE CTS Server](https://github.com/Infineon/mtb-example-btstack-freertos-cts-server).

[View this README on GitHub.](https://github.com/Infineon/mtb-example-btstack-freertos-cts-client)

[Provide feedback on this code example.](https://cypress.co1.qualtrics.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyMzAzMDQiLCJTcGVjIE51bWJlciI6IjAwMi0zMDMwNCIsIkRvYyBUaXRsZSI6IkJsdWV0b290aCZyZWc7IExFIENUUyBjbGllbnQiLCJyaWQiOiJuaGVnIiwiRG9jIHZlcnNpb24iOiI0LjQuMCIsIkRvYyBMYW5ndWFnZSI6IkVuZ2xpc2giLCJEb2MgRGl2aXNpb24iOiJNQ0QiLCJEb2MgQlUiOiJJQ1ciLCJEb2MgRmFtaWx5IjoiQlRBQkxFIn0=)

## Requirements

- [ModusToolbox&trade; software](https://www.infineon.com/modustoolbox) v3.0 or later (tested with v3.0)
- Board support package (BSP) minimum required version for:
   - PSoC&trade; 6 MCU: v4.0.0
   - CYW920829M2EVK-02: v1.0.0.Beta4
- Programming language: C
- Associated parts: All [PSoC&trade; 6 MCU](https://www.infineon.com/cms/en/product/microcontroller/32-bit-psoc-arm-cortex-microcontroller/psoc-6-32-bit-arm-cortex-m4-mcu) with [AIROC&trade; CYW43012 Wi-Fi & Bluetooth&reg; combo chip](https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/wi-fi-4-802.11n/cyw43012), [AIROC&trade; CYW4343W Wi-Fi & Bluetooth&reg; combo chip](https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/wi-fi-4-802.11n/cyw4343w), [AIROC&trade; CYW43438 Wi-Fi & Bluetooth® combo chip](https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/cyw43438), [AIROC&trade; CYW4373 Wi-Fi & Bluetooth&reg; combo chip](https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/wi-fi-5-802.11ac/cyw4373) and [AIROC™ CYW20829 Bluetooth® LE SoC](https://www.infineon.com/cms/en/product/promopages/airoc20829)

## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; embedded compiler v10.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`
- Arm&reg; compiler v6.16 (`ARM`)
- IAR C/C++ compiler v9.30.1 (`IAR`)



## Supported kits (make variable 'TARGET')

- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; prototyping kit](https://www.infineon.com/CY8CPROTO-062-4343W) (`CY8CPROTO-062-4343W`) – Default value of `TARGET`
- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; pioneer kit](https://www.infineon.com/CY8CKIT-062-WIFI-BT) (`CY8CKIT-062-WIFI-BT`)
- [PSoC&trade; 62S2 Wi-Fi Bluetooth&reg; pioneer kit](https://www.infineon.com/CY8CKIT-062S2-43012) (`CY8CKIT-062S2-43012`)
- [PSoC&trade; 62S1 Wi-Fi Bluetooth&reg; pioneer kit](https://www.infineon.com/CYW9P62S1-43438EVB-01) (`CYW9P62S1-43438EVB-01`)
- [PSoC&trade; 62S1 Wi-Fi Bluetooth&reg; pioneer kit](https://www.infineon.com/CYW9P62S1-43012EVB-01) (`CYW9P62S1-43012EVB-01`)
- [PSoC&trade; 62S3 Wi-Fi Bluetooth&reg; prototyping kit](https://www.infineon.com/CY8CPROTO-062S3-4343W) (`CY8CPROTO-062S3-4343W`)
- [PSoC&trade; 64 "Secure Boot" Wi-Fi Bluetooth&reg; pioneer kit](https://www.infineon.com/CY8CKIT-064B0S2-4343W) (`CY8CKIT-064B0S2-4343W`)
- [PSoC&trade; 62S2 evaluation kit](https://www.infineon.com/CY8CEVAL-062S2) (`CY8CEVAL-062S2`, `CY8CEVAL-062S2-LAI-4373M2`, `CY8CEVAL-062S2-MUR-43439M2`, `CY8CEVAL-062S2-LAI-43439M2`, `CY8CEVAL-062S2-MUR-4373EM2`, `CY8CEVAL-062S2-MUR-4373M2`)
- AIROC&trade; CYW20829 Bluetooth&reg; LE evaluation kit (`CYW920829M2EVK-02`)
- [PSoC&trade; 6 Bluetooth&reg; LE pioneer kit](https://www.infineon.com/CY8CKIT-062-BLE) (`CY8CKIT-062-BLE`)
- [PSoC&trade; 6 Bluetooth&reg; LE prototyping kit](https://www.infineon.com/CY8CPROTO-063-BLE) (`CY8CPROTO-063-BLE`)
- [EZ-BLE Arduino Evaluation Board](https://www.infineon.com/cms/en/product/evaluation-boards/cyble-416045-eval/) (`CYBLE-416045-EVAL`)
- [PSoC&trade; 62S2 Wi-Fi Bluetooth&reg; prototyping kit](https://www.infineon.com/CY8CPROTO-062-43439) (`CY8CPROTO-062S2-43439`)

## Hardware setup

This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

**Note:** The PSoC&trade; 6 Bluetooth&reg; LE pioneer kit (CY8CKIT-062-BLE) and the PSoC&trade; 6 Wi-Fi Bluetooth&reg; pioneer kit (CY8CKIT-062-WIFI-BT) ship with KitProg2 installed. The ModusToolbox&trade; software requires KitProg3. Before using this code example, make sure that the board is upgraded to KitProg3. The tool and instructions are available in the [Firmware Loader](https://github.com/Infineon/Firmware-loader) GitHub repository. If you do not upgrade, you will see an error like "unable to find CMSIS-DAP device" or "KitProg firmware is out of date".

The AIROC&trade; CYW20829 Bluetooth&reg; kit (CYW920829M2EVK-02) ships with KitProg3 version 2.21 installed. The ModusToolbox™ software requires KitProg3 with latest version 2.40. Before using this code example, make sure that the board is upgraded to KitProg3. The tool and instructions are available in the Firmware Loader GitHub repository. If you do not upgrade, you will see an error such as "unable to find CMSIS-DAP device" or "KitProg firmware is out of date".

## Software setup
Install a terminal emulator if you don't have one. Instructions in this document use [Tera Term](https://ttssh2.osdn.jp/index.html.en).


## Using the code example

Create the project and open it using one of the following:

<details><summary><b>In Eclipse IDE for ModusToolbox&trade; software</b></summary>

1. Click the **New Application** link in the **Quick Panel** (or, use **File** > **New** > **ModusToolbox&trade; Application**). This launches the [Project Creator](https://www.infineon.com/ModusToolboxProjectCreator) tool.

2. Pick a kit supported by the code example from the list shown in the **Project Creator - Choose Board Support Package (BSP)** dialog.

   When you select a supported kit, the example is reconfigured automatically to work with the kit. To work with a different supported kit later, use the [Library Manager](https://www.infineon.com/ModusToolboxLibraryManager) to choose the BSP for the supported kit. You can use the Library Manager to select or update the BSP and firmware libraries used in this application. To access the Library Manager, click the link from the **Quick Panel**.

   You can also just start the application creation process again and select a different kit.

   If you want to use the application for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. In the **Project Creator - Select Application** dialog, choose the example by enabling the checkbox.

4. (Optional) Change the suggested **New Application Name**.

5. The **Application(s) Root Path** defaults to the Eclipse workspace which is usually the desired location for the application. If you want to store the application in a different location, you can change the *Application(s) Root Path* value. Applications that share libraries should be in the same root path.

6. Click **Create** to complete the application creation process.

For more details, see the [Eclipse IDE for ModusToolbox&trade; software user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mtb_ide_user_guide.pdf*).

</details>

<details><summary><b>In command-line interface (CLI)</b></summary>

ModusToolbox&trade; software provides the Project Creator as both a GUI tool and the command line tool, "project-creator-cli". The CLI tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; software install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the "project-creator-cli" tool. On Windows, use the command line "modus-shell" program provided in the ModusToolbox&trade; software installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; software tools. You can access it by typing `modus-shell` in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The "project-creator-cli" tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the `<id>` field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the `<id>` field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

<br>

The following example clones the "[Bluetooth&reg; LE CTS Client](https://github.com/Infineon/mtb-example-btstack-freertos-cts-client)" application with the desired name "CTSClient" configured for the *CY8CKIT-062-WIFI-BT* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id CY8CKIT-062-WIFI-BT --app-id mtb-example-btstack-freertos-cts-client --user-app-name CTSClient --target-dir "C:/mtb_projects"
   ```

**Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; software user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mtb_user_guide.pdf*).

To work with a different supported kit later, use the [Library Manager](https://www.infineon.com/ModusToolboxLibraryManager) to choose the BSP for the supported kit. You can invoke the Library Manager GUI tool from the terminal using `make library-manager` command or use the Library Manager CLI tool "library-manager-cli" to change the BSP.

The "library-manager-cli" tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--add-bsp-name` | Name of the BSP that should be added to the application | Required
`--set-active-bsp` | Name of the BSP that should be as active BSP for the application | Required
`--add-bsp-version`| Specify the version of the BSP that should be added to the application if you do not wish to use the latest from manifest | Optional
`--add-bsp-location`| Specify the location of the BSP (local/shared) if you prefer to add the BSP in a shared path | Optional

<br>

The following example adds the CY8CPROTO-062-4343W BSP to the already created application and makes it the active BSP for the app:

   ```
   library-manager-cli --project "C:/mtb_projects/CTSClient" --add-bsp-name CY8CPROTO-062-4343W --add-bsp-version "latest-v4.X" --add-bsp-location "local"

   library-manager-cli --project "C:/mtb_projects/CTSClient" --set-active-bsp APP_CY8CPROTO-062-4343W
   ```

</details>

<details><summary><b>In third-party IDEs</b></summary>

Use one of the following options:

- **Use the standalone [Project Creator](https://www.infineon.com/ModusToolboxProjectCreator) tool:**

   1. Launch Project Creator from the Windows Start menu or from *{ModusToolbox&trade; software install directory}/tools_{version}/project-creator/project-creator.exe*.

   2. In the initial **Choose Board Support Package** screen, select the BSP, and click **Next**.

   3. In the **Select Application** screen, select the appropriate IDE from the **Target IDE** drop-down menu.

   4. Click **Create** and follow the instructions printed in the bottom pane to import or open the exported project in the respective IDE.

<br>

- **Use command-line interface (CLI):**

   1. Follow the instructions from the **In command-line interface (CLI)** section to create the application.

   2. Export the application to a supported IDE using the `make <ide>` command.

   3. Follow the instructions displayed in the terminal to create or import the application as an IDE project.

For a list of supported IDEs and more details, see the "Exporting to IDEs" section of the [ModusToolbox&trade; software user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>

## Operation

If using a PSoC&trade; 64 "Secure" MCU kit (like CY8CKIT-064B0S2-4343W), the PSoC&trade; 64 device must be provisioned with keys and policies before being programmed. Follow the instructions in the ["Secure Boot" SDK user guide](https://www.infineon.com/dgdlac/Infineon-PSoC_64_Secure_MCU_Secure_Boot_SDK_User_Guide-Software-v07_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0f8c361a7666) to provision the device. If the kit is already provisioned, copy-paste the keys and policy folder to the application folder.

1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector.

2. Open a terminal program and select the KitProg3 COM port. Set the serial port parameters to 8N1 and 115200 baud.

3. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE for ModusToolbox&trade; software</b></summary>

      1. Select the application project in the Project Explorer.

      2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)**.
   </details>

   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. The default toolchain is specified in the application's Makefile but you can override this value manually:
      ```
      make program TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TOOLCHAIN=GCC_ARM
      ```
   </details>

4. After programming, the application starts automatically. Observe the messages on the UART terminal. Use the KitProg3 COM port to view the Bluetooth&reg; stack and application trace messages in the terminal window as shown in Figure 1.

   **Figure 1. Terminal output when the device is programmed with the CE**
   ![](images/terminal_output1.png)

5. Use another supported PSoC&trade; 6 kit and program it with the [Bluetooth&reg; LE CTS Server](https://github.com/Infineon/mtb-example-btstack-freertos-cts-server) code example.

6. Press the user button (SW2) on the kit to start high duty advertisement. After 30 seconds, if the device is still not connected, it switches to low-duty advertisements without timeout.

   **Figure 2. Terminal output showing connection**

   ![](images/terminal_output2.png)

7. After connecting with the CTS Server device, press the same user button (SW2) to enable or disable notifications.

   **Figure 3. Terminal output showing notifications**

   ![](images/terminal_output3.png)

## Debugging

You can debug the example to step through the code. In the IDE, use the **\<Application Name> Debug (KitProg3_MiniProg4)** configuration in the **Quick Panel**. For details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; software user guide](https://www.infineon.com/MTBEclipseIDEUserGuide).

**Note:** **(Only while debugging PSoC&trade; 6 MCU)** On the CM4 CPU, some code in `main()` may execute before the debugger halts at the beginning of `main()`. This means that some code executes twice – once before the debugger stops execution, and again after the debugger resets the program counter to the beginning of `main()`. See [KBA231071](https://community.infineon.com/t5/Knowledge-Base-Articles/PSoC-6-MCU-Code-in-main-executes-before-the-debugger-halts-at-the-first-line-of/ta-p/253856) to learn about this and for the workaround.

## Design and implementation

The code example configures the device as a AIROC&trade; Bluetooth&reg; LE GAP Peripheral and GATT Client. Current Time Service(CTS) is showcased in the example. The device advertises with the name 'CTS Client'. After connection with the AIROC&trade; Bluetooth&reg; LE Central device, it sends a Service Discovery (by UUID) request. If the CTS UUID is present in the server GATT database, the client device enables notifications for CTS by writing into the Client Characteristic Configuration Descriptor (CCCD). The date and time notifications received are printed on the terminal.

A user button is used to start advertisement or enable/disable notifications from the server device.

The application uses a UART resource from the Hardware Abstraction Layer (HAL) to print debug messages on a UART terminal emulator. The UART resource initialization and retargeting of standard I/O to the UART port are done using the retarget-io library.

### Power measurement: Implementation of low power for AIROC&trade; Bluetooth&reg; LE

This examples enables you to measure the power consumption in three different AIROC&trade; Bluetooth&reg; LE states: Standby, Advertising, and Connected. Do the following to enter each state and the measure the power consumption for different kits.

1. **Standby state:** After programming the device, AIROC&trade; Bluetooth&reg; LE is initialized and stays in standby state. On the terminal, check for the message 'Bluetooth&reg; stack initialization successful'; you can measure the power consumption for standby state.

2. **Advertising state:** Press the user button (SW2) on your kit to start advertising. Note that the kit with the CTS Server CE must not be scanning when advertising is started because the peer will send a connection request as soon as it discovers the device. High duty advertisements start and stay for 30 seconds. After 30 seconds, the device switches to low duty advertisements. If required, this configuration can be changed using the 'bt-configurator' tool that comes with ModusToolbox&trade; installation.

3. **Connected state:** Start scanning on the CTS Server and ensure that the kit is advertising. Once the scanner finds the advertiser, connection is established. The terminal displays the message 'Connected : BDA xx:xx:xx:xx:xx:xx'.

### Current measuring points for kits

#### **CY8CKIT-062S2-43012, CYW9P62S1-43438EVB-01, CYW9P62S1-43012EVB-01 and CY8CKIT-064B0S2-4343W**

*For PSoC&trade; 6 MCU:*

1. Remove J25 to eliminate leakage currents across potentiometer R1.

2. Measure the current at J15 across VTARG and P6_VDD.

*For CYW43xxx:*

- Measure the current at VBAT across VBAT and VCC_VBAT at J8.

#### **CY8CEVAL-062S2-LAI-4373M2**

*For PSoC&trade; 6 MCU:*

1. Remove J21 to eliminate leakage currents across potentiometer R1.

2. Measure the current at J15 across VTARG and P6_VDD.

*For CYW4373E:*

- Measure the current at VBAT across VBAT and VCC_VBAT at J11.

#### **CY8CPROTO-062-4343W**

*For PSoC&trade; 6 MCU:*

1. Remove R65 on the right of the board close to the USB connector of the PSoC&trade; 6 MCU device.

2. Connect an ammeter between VTARG (J2.32) and P6_VDD (J2.24).

3. Remove R24 at the back of the board, below J1.9, to eliminate the leakage current.

   R24 is the pull-up resistor attached to the WL_HOST_WAKE pin P0_4, which leaks approximately 330 µA because P0_4 is driven LOW when there is no network activity. In total, the PSoC&trade; 6 MCU deep sleep current is approximately 350 µA.

*For CYW4343W:*

1. Measure the current at VBAT1 and VBAT2 supplies used for powering CYW4343W. VBAT1 and VABT2 are shorted to each other.

2. Remove R87 on the back of the board towards the right and above J2.33.

3. Connect an ammeter between the pads of R87 to measure the current.

#### **CY8CKIT-062-WIFI-BT**

*For PSoC&trade; 6 MCU:*

- Measure the current  by connecting an ammeter to the PWR MON jumper J8.

*For CYW4343W:*

- Measure the current at WL_VBAT (used for powering CYW4343W) by removing L3 along the right edge of the board close to the CYW4343W module, and connecting an ammeter between the pads of L3.

#### **CY8CPROTO-062S3-4343W**

*For PSoC&trade; 6 MCU:*

- Measure the current by removing R59 and connecting an ammeter across VTARG (J2.32) and P6_VDD (J2.31)

*For CYW4343W:*

- Measure the current by removing R55 and connecting an ammeter between the resistor pads (VCC_3V6 and VBAT_WL).

Table 1 captures the current numbers measured using this CE for two BSPs. The kits have different connectivity devices - CYW43012 and CYW43438. The measurement is not performed in a radio isolated environment. The current consumption by the PSoC&trade; 6 device is also measured; it was identical across all Bluetooth&reg; states. The average current value is given below:

1. For PSoC&trade; 6 device in CY8CKIT-062S2-43012: 27 µA

2. For PSoC&trade; 6 device in CYW9P62S1-43438EVB-01: 620 µA

3. For PSoC&trade; 6 device in CY8CEVAL-062S2-LAI-4373M2: 23 µA

**Table 1. Current consumption**

 |Bluetooth&reg; state|Setting |CY8CKIT-062S2-43012|CYW9P62S1-43438EVB-01|CY8CEVAL-062S2-LAI-4373M2|
 | :-------      | :------------      | :------------   | :------------ | :------------ |
 |Standby                 |Stack initialized      |3.65 mA    |4.20 mA   |8.4 mA    |
 |High duty advertisement |ADV interval: 30 ms    |1.37 mA    |2.15 mA   |3.34 mA   |
 |Low duty advertisement  |ADV interval: 1.28 s   |43 µA      |70 µA     |480 µA    |
 |Connected               |Conn interval: 8.75 ms |1.88 mA    |3.16 mA   |7.05 mA   |
 |Connected               |Conn interval: 10 ms   |1.66 mA    |2.7 mA    |5.09 mA   |
 |Connected               |Conn interval: 50 ms   |410 µA     |594 µA    |1.32 mA   |
 |Connected               |Conn interval: 500 ms  |60.17 µA   |213 µA    |652.3 µA  |
 |Connected               |Conn interval: 1 s     |46.6 µA    |69.32 µA  |495 µA    |


<br>

### Resources and settings

This section explains the ModusToolbox&trade; resources and their configuration as used in this code example. Note that all the configuration explained in this section has already been done in the code example.

- **Device Configurator:** ModusToolbox&trade; stores the configuration settings of the application in the *design.modus* file. This file is used by the Device Configurator, which generates the configuration firmware. This firmware is stored in the application’s *GeneratedSource* folder. By default, all applications in a workspace share the same *design.modus* file - i.e., they share the same pin configuration. Each BSP has a default *design.modus* file in the *mtb_shared\TARGET_\<bsp name\>\\<version\>\COMPONENT_BSP_DESIGN_MODUS* directory. 

   It is not recommended to modify the configuration of a standard BSP directly. To modify the configuration for a single application or to create a custom BSP refer to the [ModusToolbox&trade; user guide](https://www.infineon.com/dModusToolboxUserGuide). This example uses the default configuration.

   For detailed information on how to use the Device Configurator, see the [Device Configurator guide](https://www.infineon.com/dgdl/Infineon-ModusToolbox_Device_Configurator_Guide_4-UserManual-v01_00-EN.pdf?fileId=8ac78c8c7d718a49017d99ab297631cb).

- **Bluetooth&reg; Configurator:** The Bluetooth&reg; peripheral has an additional configurator called the “Bluetooth&reg; Configurator” that is used to generate the AIROC&trade; Bluetooth&reg; LE GATT database and various Bluetooth&reg; settings for the application. These settings are stored in the file named *design.cybt*. 

   Note that unlike the Device Configurator, the Bluetooth&reg; Configurator settings and files are local to each respective application.

   For detailed information on how to use the Bluetooth&reg; Configurator, see the [Bluetooth&reg; Configurator guide](https://www.infineon.com/dgdl/Infineon-ModusToolbox_Bluetooth_Configurator_Guide_3-UserManual-v01_00-EN.pdf?fileId=8ac78c8c7d718a49017d99aaf5b231be).

**Note:** For PSoC&trade; 6 Bluetooth&reg; LE-based BSPs (CY8CKIT-062-BLE, CY8CPROTO-063-BLE, CYBLE-416045-EVAL) with support for AIROC&trade; BTSTACK, if you want to use the bt-configurator tool, select the *AIROC&trade; BTSTACK with Bluetooth&reg; LE only (CYW20829, PSoC&trade; 6 with CYW43xxx Connectivity device)* option from the dropdown to select the device. Do not use the *PSoC&trade; Bluetooth&reg; LE Legacy Stack (PSoC&trade; 6-BLE)* option because it is not compatible with AIROC&trade; BTSTACK.

**Table 2. Application resources**

 Resource  |  Alias/object     |    Purpose
 :-------- | :-------------    | :------------
 UART (HAL)|cy_retarget_io_uart_obj| UART HAL object used by Retarget-IO for Debug UART port
 GPIO (HAL)| CYBSP_USER_BTN    | Start advertisement or enable/disable notification


## Related resources

Resources  | Links
-----------|----------------------------------
Application notes  | [AN228571](https://www.infineon.com/AN228571) – Getting started with PSoC&trade; 6 MCU on ModusToolbox&trade; software <br>  [AN215656](https://www.infineon.com/AN215656) – PSoC&trade; 6 MCU: Dual-CPU system design
Code examples  | [Using ModusToolbox&trade; software](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [PSoC&trade; 6 MCU datasheets](https://www.infineon.com/cms/en/search.html#!view=downloads&term=psoc6&doc_group=Data%20Sheet) <br> [PSoC&trade; 6 technical reference manuals](https://www.infineon.com/cms/en/search.html#!view=downloads&term=psoc6&doc_group=Additional%20Technical%20Information)<br>[AIROC&trade; CYW20829 Bluetooth&reg; LE SoC](https://www.infineon.com/cms/en/product/promopages/airoc20829)
Development kits | Select your kits from the [evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board)
Libraries on GitHub  | [mtb-pdl-cat1](https://github.com/Infineon/mtb-pdl-cat1) – PSoC&trade; 6 peripheral driver library (PDL)  <br> [mtb-hal-cat1](https://github.com/Infineon/mtb-hal-cat1) – Hardware abstraction layer (HAL) library <br> [retarget-io](https://github.com/Infineon/retarget-io) – Utility library to retarget STDIO messages to a UART port
Middleware on GitHub  | [capsense](https://github.com/Infineon/capsense) – CAPSENSE&trade; library and documents <br> [psoc6-middleware](https://github.com/Infineon/modustoolbox-software#psoc-6-middleware-libraries) – Links to all PSoC&trade; 6 MCU middleware
Tools  | [Eclipse IDE for ModusToolbox&trade; software](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use software and tools enabling rapid development with Infineon MCUs, covering applications from embedded sense and control to wireless and cloud-connected systems using AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices.

<br>

## Other resources

Infineon provides a wealth of data at www.infineon.com to help you select the right device, and quickly and effectively integrate it into your design.

For PSoC&trade; 6 MCU devices, see [How to design with PSoC&trade; 6 MCU – KBA223067](https://community.infineon.com/t5/Knowledge-Base-Articles/How-to-Design-with-PSoC-6-MCU-KBA223067/ta-p/248857) in the Infineon Developer Community.

## Document history

Document title: *CE230304* – *Bluetooth&reg; LE CTS Client*

 Version | Description of change
 ------- | ---------------------
 1.0.0   | New code example    
 2.0.0   | Major update to support ModusToolbox&trade; software v2.3.1,<br>This version is not backward compatible with ModusToolbox&trade; software v2.2 or older versions, <br>Added support for new kits </br> Enabled for CYW43xxx Low power and addition of current number
 3.0.0   | Added support for CYW43439 kit <br> Updated BSP to 3.0.0
 4.0.0   | Updated to support ModusToolbox&trade; software v3.0 and BSPs v4.x
 4.1.0   | Added support for CYW920829M2EVB-01,CY8CKIT-062-BLE,CY8CPROTO-063-BLE, CYBLE-416045-EVAL
 4.2.0   | Added support for CY8CEVAL-062S2-LAI-43439M2,CY8CPROTO-062S2-43439
 4.3.0   | Removed CYW920829M2EVB-01 from supported kits <br> Added support for CYW920829M2EVK-02
 4.4.0   | Added support for CY8CEVAL-062S2-MUR-4373EM2 and CY8CEVAL-062S2-MUR-4373M2
------

---------------------------------------------------------

© Cypress Semiconductor Corporation, 2020-2023. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress’s patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br>
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product. CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications. To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document. Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device. You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device. Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress’s published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, WICED, ModusToolbox, PSoC, CapSense, EZ-USB, F-RAM, and Traveo are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries. For a more complete list of Cypress trademarks, visit www.infineon.com. Other names and brands may be claimed as property of their respective owners.

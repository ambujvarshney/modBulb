# Created by Microsemi Libero Software 11.7.0.119
# Fri Nov 11 11:41:28 2016

# (NEW DESIGN)

# create a new design
new_design -name "Top" -family "IGLOO"
set_device -die {AGLN060V5} -package {100 VQFP} -speed {STD} -voltage {1.5} -IO_DEFT_STD {LVCMOS 1.5V} -RESERVEMIGRATIONPINS {1} -RESTRICTPROBEPINS {1} -RESTRICTSPIPINS {0} -TARGETDEVICESFORMIGRATION {UM2X2M1NLP} -TEMPR {COM} -UNUSED_MSS_IO_RESISTOR_PULL {None} -VCCI_1.5_VOLTR {COM} -VCCI_1.8_VOLTR {COM} -VCCI_2.5_VOLTR {COM} -VCCI_3.3_VOLTR {COM} -VOLTR {COM}


# set default back-annotation base-name
set_defvar "BA_NAME" "Top_ba"
set_defvar "IDE_DESIGNERVIEW_NAME" {Impl1}
set_defvar "IDE_DESIGNERVIEW_COUNT" "1"
set_defvar "IDE_DESIGNERVIEW_REV0" {Impl1}
set_defvar "IDE_DESIGNERVIEW_REVNUM0" "1"
set_defvar "IDE_DESIGNERVIEW_ROOTDIR" {C:\Users\Abdullah\Projects\FPGA\Workplace\ModBulb_modulation_FPGA\designer}
set_defvar "IDE_DESIGNERVIEW_LASTREV" "1"

# set working directory
set_defvar "DESDIR" "C:/Users/Abdullah/Projects/FPGA/Workplace/ModBulb_modulation_FPGA/designer/impl1"

# set back-annotation output directory
set_defvar "BA_DIR" "C:/Users/Abdullah/Projects/FPGA/Workplace/ModBulb_modulation_FPGA/designer/impl1"

# enable the export back-annotation netlist
set_defvar "BA_NETLIST_ALSO" "1"

# set EDIF options
set_defvar "EDNINFLAVOR" "GENERIC"

# set HDL options
set_defvar "NETLIST_NAMING_STYLE" "VHDL93"

# setup status report options
set_defvar "EXPORT_STATUS_REPORT" "1"
set_defvar "EXPORT_STATUS_REPORT_FILENAME" "Top.rpt"

# legacy audit-mode flags (left here for historical reasons)
set_defvar "AUDIT_NETLIST_FILE" "1"
set_defvar "AUDIT_DCF_FILE" "1"
set_defvar "AUDIT_PIN_FILE" "1"
set_defvar "AUDIT_ADL_FILE" "1"

# import of input files
import_source  \
-format "edif" -edif_flavor "GENERIC" -netlist_naming "VHDL" {../../synthesis/Top.edn}

# save the design database
save_design {Top.adb}


compile
report -type "status" {Top_compile_report.txt}
report -type "pin" -listby "name" {Top_report_pin_byname.txt}
report -type "pin" -listby "number" {Top_report_pin_bynumber.txt}

save_design

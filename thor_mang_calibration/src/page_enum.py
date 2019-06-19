# enumeration and specification of all pages used by the calibration wizard
# pages 'Intro', 'Pose', 'Walking_Calibration', and 'Summary' are pages with special UIs and function
# all other pages are of the normal calibration_page type and may be added at will
# names need to be the same as the specified joints in information

from enum import Enum

Pages = Enum('Pages','Intro Pose L_Leg_Hip R_Leg_Hip L_Leg_Knee R_Leg_Knee L_Leg_Ankle R_Leg_Ankle Torso Head L_Arm_Shoulder L_Arm_Elbow L_Arm_Wrist  R_Arm_Shoulder R_Arm_Elbow R_Arm_Wrist Walking_Calibration Summary')

#include <Arduino.h>
#include "Magnetometer.h"
   
   //-----------------mag-stuff------------------------//

        float magnetometer_xy = 0.7071 * (magnetometer_x + magnetometer_y);
		
        comp_xy = (comp_xy * alpha_xy) + (magnetometer_xy * (1-alpha_xy));
        float filtered_xy = magnetometer_xy - comp_xy;

        comp_z = (comp_z * alpha_z) + (magnetometer_z * (1-alpha_z));
        float filtered_z = magnetometer_z - comp_z;

        angle = atan2(filtered_xy, filtered_z);
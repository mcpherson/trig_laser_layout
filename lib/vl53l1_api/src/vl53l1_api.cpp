/* vl53l1_api library by Marlon McPherson
 */

#include "vl53l1_api.h"

/**
 * Constructor.
 */
Vl53l1_api::Vl53l1_api()
{
  // be sure not to call anything that requires hardware be initialized here, put those in begin()
}

/**
 * Example method.
 */
void Vl53l1_api::begin()
{
    // initialize hardware
    Serial.println("called begin");
}

/**
 * Example method.
 */
void Vl53l1_api::process()
{
    // do something useful
    Serial.println("called process");
    doit();
}

/**
* Example private method
*/
void Vl53l1_api::doit()
{
    Serial.println("called doit");
}

/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#define STRATEGY true
#define MAX_VEL 40.0f
#define MIN_VEL 2.0f

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.01f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {
   bool new_strategy = STRATEGY;

   if (!new_strategy) {
     /* Get readings from proximity sensor */
     const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
     /* Sum them together */
     CVector2 cAccumulator;
     for(size_t i = 0; i < tProxReads.size(); ++i) {
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
     }
     cAccumulator /= tProxReads.size();
     /* If the angle of the vector is small enough and the closest obstacle
      * is far enough, continue going straight, otherwise curve a little
      */
     CRadians cAngle = cAccumulator.Angle();
     if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
        cAccumulator.Length() < m_fDelta ) {
        /* Go straight */
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
     }
     else {
        /* Turn, depending on the sign of the angle */
        if(cAngle.GetValue() > 0.0f) {
           m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
        }
        else {
           m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
        }
     }
   } else {
     /* Get readings from proximity sensor */
     const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
     /* Sum them together */
     CVector2 cAccumulator;
     for(size_t i = 0; i < tProxReads.size(); ++i) {
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
     }
     cAccumulator /= tProxReads.size();
     /* If the angle of the vector is small enough and the closest obstacle
      * is far enough, continue going straight, otherwise curve a little
      */
     CRadians cAngle = cAccumulator.Angle();
     if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
        cAccumulator.Length() < m_fDelta ) {

        // This condition is added to avoid the algorithm not detecting obstacles when the robot is heading stratight onto them.
        if (cAccumulator.GetX() != 0.0f) {
          m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity * 0.9f);
        } else {
          /* Go straight */
          // If we do not detect obstacles, we can accelerate up to a maximum velocity.
          m_fWheelVelocity = 1.5f * m_fWheelVelocity;
          m_fWheelVelocity = (m_fWheelVelocity > MAX_VEL) ? MAX_VEL : m_fWheelVelocity;
          m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
        }

     }
     else {
        // If we detect an obstacle, we slow down up to a minimum Velocity.
        m_fWheelVelocity = 0.95f * m_fWheelVelocity;
        m_fWheelVelocity = (m_fWheelVelocity < MIN_VEL) ? MIN_VEL : m_fWheelVelocity;
        /* Turn, depending on the sign of the angle */

        // If the obstacles are mostly behind us, we don't need to turn as much
        if (abs(cAngle.GetValue()) > 2.0f) {
          m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity * 0.8f);
        }
        // Else, turn around to get "as far as possible" from the obstcale.
        // The damping factor of 0.8 is to avoid completely stopping to turn around. 
        else if(cAngle.GetValue() > 0.0f) {
           m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity * 0.8f);
        }
        else {
           m_pcWheels->SetLinearVelocity(-m_fWheelVelocity * 0.8f, m_fWheelVelocity);
        }
     }
   }

}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")

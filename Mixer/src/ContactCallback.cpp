#include "ContactCallback.h"

void ContactCallback::OnAddContact(const chrono::collision::ChCollisionInfo& contactinfo,
                                           chrono::ChMaterialComposite* const material)
{
    // Downcast to appropriate composite material type
    auto mat = static_cast<chrono::ChMaterialCompositeNSC* const>(material);
    float GLOBAL_friction = 0.3f;
    float GLOBAL_cohesion = 0;
    float GLOBAL_compliance = 0;
    float GLOBAL_dampingf = 0.1f;

    // Set friction according to user setting:
    mat->static_friction = GLOBAL_friction;

    // Set compliance (normal and tangential at once)
    mat->compliance = GLOBAL_compliance;
    mat->complianceT = GLOBAL_compliance;
    mat->dampingf = GLOBAL_dampingf;

    // Set cohesion according to user setting:
    // Note that we must scale the cohesion force value by time step, because
    // the material 'cohesion' value has the dimension of an impulse.
    float my_cohesion_force = GLOBAL_cohesion;
    mat->cohesion = (float)msystem->GetStep() * my_cohesion_force;  //<- all contacts will have this cohesion!

    if (contactinfo.distance > 0.12)
        mat->cohesion = 0;

    // Note that here you might decide to modify the cohesion
    // depending on object sizes, type, time, position, etc. etc.
    // For example, after some time disable cohesion at all, just
    // add here:
    //    if (msystem->GetChTime() > 10) mat->cohesion = 0;
}

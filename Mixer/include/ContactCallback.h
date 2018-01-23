#ifndef CONTACTCALLBACK_H_
#define CONTACTCALLBACK_H_
#include <chrono/physics/ChSystemNSC.h>

class ContactCallback : public chrono::ChContactContainer::AddContactCallback
{
  public:
    virtual void OnAddContact(const chrono::collision::ChCollisionInfo& contactinfo,
                              chrono::ChMaterialComposite* const material) override;

    chrono::ChSystemNSC* msystem;
};


#endif

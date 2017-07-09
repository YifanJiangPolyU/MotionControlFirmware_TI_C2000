
#include "ObjectDictionary.h"
#include "SystemWarehouse.h"

void ObjectDictionary::InitObd(void){

  _InstanceArray[2] = static_cast<ObjectDictionaryEntryBase*>(SystemWarehouse::_CurrentLoopController_GetInstance());
  _AccessFunctionArray[2] = static_cast<void (ObjectDictionaryEntryBase::*)(ObdAccessHandle*)> (&CurrentLoopController::AccessCurrentLoopGains_Kp);

}

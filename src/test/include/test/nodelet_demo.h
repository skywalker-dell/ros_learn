#include <nodelet/nodelet.h>

namespace example_pkg{

class MyNodeClass : public nodelet::Nodelet
{
public:
    virtual void onInit();
};

}
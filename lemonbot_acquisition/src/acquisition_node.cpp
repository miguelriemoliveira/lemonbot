#include <lemonbot_acquisition/acquisition_node.h>

AcquisitionNode::AcquisitionNode(Params params, Options opts)
  : _params(params), _opts(opts), _ptu_client(opts.ptu_topic, true)
{
}

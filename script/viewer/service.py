#!/usr/bin/env python
from openvino_msgs.srv import *
import rclpy
import sys
from pipeTree import TreeNode
def getVersion():
    return sys.version_info.major

def getMaping(pipeline):
    map_dict = dict()
    for pipeline in pipeline.connections:
        if pipeline.input not in map_dict.keys():
            map_dict[pipeline.input] = list()
        map_dict[pipeline.input].append(pipeline.output)

    return map_dict

def getTreeByMap(parent,input,map):
    if input not in map.keys():
        return parent
    for output in map[input]:
        child = parent.add_child(output)
        getTreeByMap(child,output,map)
    return parent


def getTree(parent,input,pipeline):
    map = getMaping(pipeline)
    return getTreeByMap(parent,input,map)

def reqPipelineService(cmd, value):
    while not cli.wait_for_service(timeout_sec=1.0):
        print('Wait for service...')
    try: 
        req_msg = PipelineSrv.Request()
        req_msg.pipeline_request.cmd = cmd
        req_msg.pipeline_request.value = value
        future = cli.call_async(req_msg)
        rclpy.spin_until_future_complete(node, future)
        return future.result()
    except:
        print("Service call failed")

def usage():
    return "%s <cmd> <value>"%sys.argv[0]


rclpy.init(args=None)
node = rclpy.create_node('pipeline_client')
cli = node.create_client(PipelineSrv, '/openvino_toolkit/pipeline_service')


if __name__ == "__main__":
    if len(sys.argv) == 3:
        cmd = str(sys.argv[1])
        value = str(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print(getVersion())

    print("Requesting cmd:%s with value:%s"%(cmd, value))
    response = reqPipelineService(cmd, value)
  
    for pipeline in response.pipelines:
         root = getTree(TreeNode(pipeline.name),'',pipeline)
         root.dump()


    # node.destroy_node()
    # rclpy.shutdown()

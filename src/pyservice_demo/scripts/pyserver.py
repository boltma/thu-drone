#!/usr/bin/env python
#coding=UTF-8

# 上面指定编码utf-8，使python能够识别中文

import rospy
from pyservice_demo.srv import *

def server():
    rospy.init_node('greeting_pyserver', anonymous = False)
    srv = rospy.Service('greetings', Greeting, handle_function)   #定义程序的Server端
    rospy.loginfo('Ready to handle the request:')
    rospy.spin()

def handle_function(req):
	rospy.loginfo('Request from %s with age %d', req.name, req.age)
	# 返回一个Service_demoResponse实例化对象，其实就是返回一个response的对象，其包含的内容为我们再Service_demo.srv中定义的
    # response部分的内容，我们定义了一个string类型的变量，因此，此处实例化时传入字符串即可
	return GreetingResponse("Hi %s. I'm server!" % req.name)

# 如果单独运行此文件，则将上面定义的server_srv作为主函数运行
if __name__ == '__main__':
	server()
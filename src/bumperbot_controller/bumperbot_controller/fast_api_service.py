#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

import uvicorn
import threading
import json

from typing import Optional
from pydantic import BaseModel

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from bumperbot_msgs.msg import BumperbotStats


app = FastAPI()

origins = [
    "http://localhost",
    "http://localhost:8000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class Response(BaseModel):
  msg: str

class SimpleSpeed(BaseModel):
  linear: float
  angular: float

class FastApiService(Node):
  def __init__(self):
    super().__init__('fast_api_service')
    self.publisher_ = self.create_publisher(String, "move", 10)

    # odometry listener
    self.bumperbot_0_stats_listener_ = self.create_subscription(BumperbotStats, "/bumperbot_0/bumperbot_stats", self.bumperbot_0_stats_callback, 10)
    self.bumperbot_0_speed_publisher_ = self.create_publisher(TwistStamped, "/bumperbot_0/bumperbot_controller/cmd_vel", 10)

    self.bumperbot_0_stats_ = BumperbotStats()

    self.i_ = 0
    self.get_logger().info("starting service")

    @app.get('/test', response_model = Response)
    async def publish(): 
      msg = String()
      msg.data = 'hello world %d' % self.i_
      self.publisher_.publish(msg)
      self.get_logger().info("Publishing: %s" % msg.data)
      self.i_ += 1

      response = {
        "msg": ''
      }

      response['msg'] = "Message Published %d" % self.i_
      return response
    
    @app.get('/stats/bumperbot_0', response_model = Response)
    async def stats():
      self.get_logger().info("%s stats: x: %f, y: %f, theta: %f, linear: %f, angular: %f " % (self.bumperbot_0_stats_.name, self.bumperbot_0_stats_.x, self.bumperbot_0_stats_.y, self.bumperbot_0_stats_.theta, self.bumperbot_0_stats_.linear, self.bumperbot_0_stats_.angular))

      msg = json.dumps({
        "name": self.bumperbot_0_stats_.name,
        "x": self.bumperbot_0_stats_.x,
        "y": self.bumperbot_0_stats_.y,
        "theta": self.bumperbot_0_stats_.theta,
        "linear": self.bumperbot_0_stats_.linear,
        "angular": self.bumperbot_0_stats_.angular
      })

      response = {
        'msg': msg
      }
      return response
    
    @app.post("/move/bumperbot_0")
    async def move(move_data: SimpleSpeed):
      self.get_logger().info("linear: %f, angular: %f" % (move_data.linear, move_data.angular))

      self.publish_speed(move_data)

      response = {
        'msg': 'success'
      }

      return response
      
    
  def bumperbot_0_stats_callback(self, msg):
    self.bumperbot_0_stats_ = msg

  def publish_speed(self, move_data: SimpleSpeed):
    msg = TwistStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.twist.linear.x = move_data.linear
    msg.twist.angular.z = move_data.angular

    self.bumperbot_speed_publisher_.publish(msg)


    
def main(args=None):
  rclpy.init()
  fast_api_service = FastApiService()
  spin_thread = threading.Thread(target=rclpy.spin, args=(fast_api_service,))
  spin_thread.start()

  uvicorn.run(app, port=5000, log_level='warning')

  fast_api_service.destroy_node()
  rclpy.shutdown()

if __name__ == ('__main__'):
  main()



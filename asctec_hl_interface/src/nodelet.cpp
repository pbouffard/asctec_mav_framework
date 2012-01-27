/*

 Copyright (c) 2011, Markus Achtelik, ASL, ETH Zurich, Switzerland
 You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <boost/shared_ptr.hpp>
#include "comm.h"
#include "hl_interface.h"
#include "ssdk_interface.h"
#include "ekf_interface.h"

namespace asctec_hl_interface
{

class HLInterfaceNodelet : public nodelet::Nodelet
{
public:
  HLInterfaceNodelet()
  {

  }

private:
  HLInterface* asctecInterface;
  SSDKInterface* ssdk_if;
  EKFInterface* ekf_if;
  //Comm commp;
  //CommPtr commp2;
  Comm comm_;
  CommPtr comm;
  void onInit()
  {

    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle pnh = getPrivateNodeHandle();
    std::string port, portRX, portTX;
    int baudrate;
    //CommPtr comm(new Comm);
    //commp = new Comm();
    comm = boost::make_shared<Comm>(comm_); //boost::make_shared<Comm>(commp);

    comm->Init();

    bool connected = false;

    pnh.param("baudrate", baudrate, HLI_DEFAULT_BAUDRATE);

    std::cout << "In HLInterfaceNodelet::onInit()" << std::endl;


    pnh.param("serial_port", port, std::string("/dev/ttyUSB0"));
    NODELET_INFO_STREAM("Connecting to " << port << " at " << baudrate << " baud...");
    connected = comm->connect(port, port, baudrate);

    if (!connected)
    {
      NODELET_ERROR("unable to connect");
      assert(false);
    }
    else
      NODELET_INFO("Connected!");

    asctecInterface = new HLInterface(nh, pnh, comm);
    // disable ssdk for now, having ROS param namespace issues and I don't plan to
    // use it anyway..
    //ssdk_if = new SSDKInterface(nh, pnh, comm);
    ekf_if = new EKFInterface(nh, pnh, comm);

  }
};
PLUGINLIB_DECLARE_CLASS(asctec_hl_interface, HLInterfaceNodelet, asctec_hl_interface::HLInterfaceNodelet, nodelet::Nodelet)

}

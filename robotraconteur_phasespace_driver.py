import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import general_robotics_toolbox as rox
import argparse
import sys
import threading
import numpy as np
import time
from RobotRaconteurCompanion.Util.SensorDataUtil import SensorDataUtil

from phasespace_lib import owl

class PhaseSpaceDriver(object):
    def __init__(self,serveraddr='127.0.0.1') -> None:
        
        ## setup OWL obj
        self.streaming_client = owl.Context()
        self.streaming_client.open(serveraddr, "timeout=10000000")
        # initialize session
        self.streaming_client.initialize("streaming=1")

        ## mocap data and RR
        self._fiducials=RRN.GetStructureType('com.robotraconteur.fiducial.RecognizedFiducials')
        self._fiducial=RRN.GetStructureType('com.robotraconteur.fiducial.RecognizedFiducial')
        self._fiducials_sensor_data=RRN.GetStructureType('com.robotraconteur.fiducial.FiducialSensorData')
        self._namedposecovtype = RRN.GetStructureType('com.robotraconteur.geometry.NamedPoseWithCovariance')
        self._namedposetype = RRN.GetStructureType('com.robotraconteur.geometry.NamedPose')
        self._posetype = RRN.GetNamedArrayDType('com.robotraconteur.geometry.Pose')
        self._sensordatatype = RRN.GetStructureType('com.robotraconteur.sensordata.SensorDataHeader')
        self._tstype = RRN.GetPodDType('com.robotraconteur.datetime.TimeSpec2')
        self._sensor_data_util = SensorDataUtil(RRN)
        self.current_fiducials_sensor_data=None

        ## streaming thread setup
        self._lock = threading.RLock()
        self._streaming = False

    def srv_start_driver(self):
        
        ## running streaming thread
        running_flag=True

        print("\n")
        print("Phasespace RR Service Ready...")
    
    def send_sensor_data(self):

        # main loop
        evt = None
        while evt or (self.streaming_client.isOpen() and self.streaming_client.property("initialized")):
            
            # poll for events with a timeout (microseconds)
            evt = self.streaming_client.nextEvent(1000000)
            # nothing received, keep waiting
            if not evt: continue
            else: print(evt)
            
            # clear previous list
            fiducials = self._fiducials()
            fiducials.recognized_fiducials=[]

            # process event
            if evt.type_id == owl.Type.FRAME:
                ## get rigid body
                if "rigids" in evt:
                    for evt_rig in evt.rigids:
                        rec_fiducials = self._fiducial()
                        rigid_body = mocap_data.rigid_body_data.rigid_body_list[i]
                        rec_fiducials.fiducial_marker = 'rigid'+str(int(rigid_body.id_num))
                        rec_fiducials.pose = self._namedposecovtype()
                        rec_fiducials.pose.pose = self._namedposetype()
                        rec_fiducials.pose.pose.pose = np.zeros((1,),dtype=self._posetype)
                        rec_fiducials.pose.pose.pose[0]['position']['x'] = rigid_body.pos[0]*1000 ## mm
                        rec_fiducials.pose.pose.pose[0]['position']['y'] = rigid_body.pos[1]*1000 ## mm
                        rec_fiducials.pose.pose.pose[0]['position']['z'] = rigid_body.pos[2]*1000 ## mm
                        quat = [rigid_body.rot[3],rigid_body.rot[0],rigid_body.rot[1],rigid_body.rot[2]]
                        rec_fiducials.pose.pose.pose[0]['orientation']['w'] = quat[0]
                        rec_fiducials.pose.pose.pose[0]['orientation']['x'] = quat[1]
                        rec_fiducials.pose.pose.pose[0]['orientation']['y'] = quat[2]
                        rec_fiducials.pose.pose.pose[0]['orientation']['z'] = quat[3]
                        fiducials.recognized_fiducials.append(rec_fiducials)
                
                ## get markers
                if "markers" in evt:
                    # print(len(mocap_data.labeled_marker_data.labeled_marker_list))
                    for evt_mkr in evt.markers:
                        rec_fiducials = self._fiducial()
                        model_id,marker_id = lbmarker.get_marker_id()
                        rec_fiducials.fiducial_marker = 'marker'+str(int(marker_id))+'_rigid'+str(int(model_id))
                        rec_fiducials.pose = self._namedposecovtype()
                        rec_fiducials.pose.pose = self._namedposetype()
                        rec_fiducials.pose.pose.pose = np.zeros((1,),dtype=self._posetype)
                        rec_fiducials.pose.pose.pose[0]['position']['x'] = lbmarker.pos[0]*1000 ## mm
                        rec_fiducials.pose.pose.pose[0]['position']['y'] = lbmarker.pos[1]*1000 ## mm
                        rec_fiducials.pose.pose.pose[0]['position']['z'] = lbmarker.pos[2]*1000 ## mm
                        fiducials.recognized_fiducials.append(rec_fiducials)

                fiducials_sensor_data = self._fiducials_sensor_data()
                fiducials_sensor_data.sensor_data = self._sensordatatype()
                fiducials_sensor_data.sensor_data.seqno = int(mocap_data.prefix_data.frame_number)
                nanosec = mocap_data.suffix_data.stamp_data_received*100
                fiducials_sensor_data.sensor_data.ts = np.zeros((1,),dtype=self._tstype)
                fiducials_sensor_data.sensor_data.ts[0]['nanoseconds'] = int(nanosec%1e9)
                fiducials_sensor_data.sensor_data.ts[0]['seconds'] = int(nanosec/1e9)
                fiducials_sensor_data.fiducials = fiducials

                self.fiducials_sensor_data.AsyncSendPacket(fiducials_sensor_data, lambda: None)
                self.current_fiducials_sensor_data = fiducials_sensor_data
                
            elif evt.type_id == owl.Type.ERROR:
                # handle errors
                print(evt.name, evt.data)
                if evt.name == "fatal":
                    break
            elif evt.name == "done":
                # done event is sent when master connection stops session
                print("done")
                break
        
        running_flag=False

    def srv_stop_streaming(self):

        # self._streaming = False
        # self.data_t.join()
        self.streaming_client.shutdown()
    
    def capture_fiducials(self):

        if self.current_fiducials_sensor_data is not None:
            return self.current_fiducials_sensor_data.fiducials
        else:
            return self._fiducials()
    
    def print_configuration(self):

        print("Connection Configuration:")
        print("  Client:          %s"% self.streaming_client.local_ip_address)
        print("  Server:          %s"% self.streaming_client.server_ip_address)
        print("  Command Port:    %d"% self.streaming_client.command_port)
        print("  Data Port:       %d"% self.streaming_client.data_port)

        if self.streaming_client.use_multicast:
            print("  Using Multicast")
            print("  Multicast Group: %s"% self.streaming_client.multicast_address)
        else:
            print("  Using Unicast")

        #NatNet Server Info
        application_name = self.streaming_client.get_application_name()
        nat_net_requested_version = self.streaming_client.get_nat_net_requested_version()
        nat_net_version_server = self.streaming_client.get_nat_net_version_server()
        server_version = self.streaming_client.get_server_version()

        print("  NatNet Server Info")
        print("    Application Name %s" %(application_name))
        print("    NatNetVersion  %d %d %d %d"% (nat_net_version_server[0], nat_net_version_server[1], nat_net_version_server[2], nat_net_version_server[3]))
        print("    ServerVersion  %d %d %d %d"% (server_version[0], server_version[1], server_version[2], server_version[3]))
        print("  NatNet Bitstream Requested")
        print("    NatNetVersion  %d %d %d %d"% (nat_net_requested_version[0], nat_net_requested_version[1],\
        nat_net_requested_version[2], nat_net_requested_version[3]))

def main():
    parser = argparse.ArgumentParser(description="Phasespace Mocap driver service for Robot Raconteur")
    parser.add_argument("--server-ip", type=str, default="127.0.0.1", help="the ip address of the Phasespace server")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")

    args,_ = parser.parse_known_args()

    # not yet know what this do
    rr_args = ["--robotraconteur-jumbo-message=true"] + sys.argv
    RRC.RegisterStdRobDefServiceTypes(RRN)

    phasespace_obj = PhaseSpaceDriver(args.server_ip)

    with RR.ServerNodeSetup("com.robotraconteur.fiducial.FiducialSensor",59823,argv=rr_args):
        
        service_ctx = RRN.RegisterService("phasespace_mocap","com.robotraconteur.fiducial.FiducialSensor",phasespace_obj)
        phasespace_obj.srv_start_driver()

        if args.wait_signal:  
            #Wait for shutdown signal if running in service mode          
            print("Press Ctrl-C to quit...")
            import signal
            signal.sigwait([signal.SIGTERM,signal.SIGINT])
        
        else:    
            #Wait for the user to shutdown the service
            if (sys.version_info > (3, 0)):
                input("Server started, press enter to quit...")
            else:
                raw_input("Server started, press enter to quit...")

        phasespace_obj.srv_stop_streaming()

if __name__ == "__main__":
    main()
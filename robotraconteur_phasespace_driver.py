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
    def __init__(self,serveraddr='127.0.0.1',tcp_udp=True) -> None:
        
        ## setup OWL obj
        self.streaming_client = owl.Context()
        self.streaming_client.open(serveraddr, "timeout=10000000")
        # initialize session
        if tcp_udp:
            self.streaming_client.initialize("streaming=1")
        else:
            self.streaming_client.initialize("streaming=2")

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
        
        ## get trackers (marker's associated rigid body)
        self.mkr_tracker={}
        tracker_cnt=-1
        total_tracker_cnt=10
        while True:
            tracker_cnt+=1
            tinfo=self.streaming_client.trackerInfo(tracker_cnt)
            if tinfo is None:
                if tracker_cnt>total_tracker_cnt:
                    break
                continue
            for mkr_id in tinfo.marker_ids:
                self.mkr_tracker[mkr_id]=tinfo.id
        
        ## Start streaming thread
        self.stream_thread=threading.Thread(target=self.send_sensor_data,daemon=True)
        self.stream_thread.start()
        
        ## running streaming thread
        self._streaming=True
        self.seqno=0

        print("\n")
        print("Phasespace RR Service Ready...")
    
    def send_sensor_data(self):

        while not self._streaming:
            time.sleep(0.01)
            continue

        # main loop
        evt = None
        while self._streaming and (evt or (self.streaming_client.isOpen() and self.streaming_client.property("initialized"))):
            
            # poll for events with a timeout (microseconds)
            evt = self.streaming_client.nextEvent(1000000)
            # nothing received, keep waiting
            if not evt: continue
            # else: print(evt)
            
            # clear previous list
            fiducials = self._fiducials()
            fiducials.recognized_fiducials=[]

            # process event
            if evt.type_id == owl.Type.FRAME:
                ## get rigid body
                if "rigids" in evt:
                    for evt_rig in evt.rigids:
                        if evt_rig.cond>0:
                            rec_fiducials = self._fiducial()
                            rec_fiducials.fiducial_marker = 'rigid'+str(int(evt_rig.id))
                            rec_fiducials.pose = self._namedposecovtype()
                            rec_fiducials.pose.pose = self._namedposetype()
                            rec_fiducials.pose.pose.pose = np.zeros((1,),dtype=self._posetype)
                            rec_fiducials.pose.pose.pose[0]['position']['x'] = evt_rig.pose[0] ## mm
                            rec_fiducials.pose.pose.pose[0]['position']['y'] = evt_rig.pose[1] ## mm
                            rec_fiducials.pose.pose.pose[0]['position']['z'] = evt_rig.pose[2] ## mm
                            quat = evt_rig.pose[3:]
                            rec_fiducials.pose.pose.pose[0]['orientation']['w'] = quat[0]
                            rec_fiducials.pose.pose.pose[0]['orientation']['x'] = quat[1]
                            rec_fiducials.pose.pose.pose[0]['orientation']['y'] = quat[2]
                            rec_fiducials.pose.pose.pose[0]['orientation']['z'] = quat[3]
                            rec_fiducials.confidence = evt_rig.cond
                            fiducials.recognized_fiducials.append(rec_fiducials)
                
                ## get markers
                if "markers" in evt:
                    # print(len(mocap_data.labeled_marker_data.labeled_marker_list))
                    for evt_mkr in evt.markers:
                        if evt_mkr.cond>0:
                            if evt_mkr.id in self.mkr_tracker.keys(): model_id=self.mkr_tracker[evt_mkr.id]
                            else: model_id=0
                            rec_fiducials = self._fiducial()
                            rec_fiducials.fiducial_marker = 'marker'+str(int(evt_mkr.id))+'_rigid'+str(int(model_id))
                            rec_fiducials.pose = self._namedposecovtype()
                            rec_fiducials.pose.pose = self._namedposetype()
                            rec_fiducials.pose.pose.pose = np.zeros((1,),dtype=self._posetype)
                            rec_fiducials.pose.pose.pose[0]['position']['x'] = evt_mkr.x ## mm
                            rec_fiducials.pose.pose.pose[0]['position']['y'] = evt_mkr.y ## mm
                            rec_fiducials.pose.pose.pose[0]['position']['z'] = evt_mkr.z ## mm
                            rec_fiducials.confidence = evt_mkr.cond
                            fiducials.recognized_fiducials.append(rec_fiducials)

                fiducials_sensor_data = self._fiducials_sensor_data()
                fiducials_sensor_data.sensor_data = self._sensordatatype()
                fiducials_sensor_data.sensor_data.seqno = int(self.seqno)
                nanosec = evt.time*1000
                fiducials_sensor_data.sensor_data.ts = np.zeros((1,),dtype=self._tstype)
                fiducials_sensor_data.sensor_data.ts[0]['nanoseconds'] = int(nanosec%1e9)
                fiducials_sensor_data.sensor_data.ts[0]['seconds'] = int(nanosec/1e9)
                fiducials_sensor_data.fiducials = fiducials

                self.fiducials_sensor_data.AsyncSendPacket(fiducials_sensor_data, lambda: None)
                self.current_fiducials_sensor_data = fiducials_sensor_data
                
                self.seqno+=1
                
            elif evt.type_id == owl.Type.ERROR:
                # handle errors
                print(evt.name, evt.data)
                if evt.name == "fatal":
                    break
            elif evt.name == "done":
                # done event is sent when master connection stops session
                print("done")
                break

    def srv_stop_streaming(self):

        self._streaming = False
        self.stream_thread.join()
        # end session
        self.streaming_client.done()
        # close socket
        self.streaming_client.close()
    
    def capture_fiducials(self):

        if self.current_fiducials_sensor_data is not None:
            return self.current_fiducials_sensor_data.fiducials
        else:
            return self._fiducials()

def main():
    parser = argparse.ArgumentParser(description="Phasespace Mocap driver service for Robot Raconteur")
    parser.add_argument("--server-ip", type=str, default="127.0.0.1", help="The ip address of the Phasespace server")
    parser.add_argument("--tcp", type=bool, default=True, help="TCP (True) or UDP (False). Default: TCP")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")

    args,_ = parser.parse_known_args()

    # not yet know what this do
    rr_args = ["--robotraconteur-jumbo-message=true"] + sys.argv
    RRC.RegisterStdRobDefServiceTypes(RRN)

    phasespace_obj = PhaseSpaceDriver(args.server_ip,args.tcp)

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
        print("PhaseSpace RR Sever End.")

if __name__ == "__main__":
    main()
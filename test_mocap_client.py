from RobotRaconteur.Client import *
import time
import numpy

def main():
    url='rr+tcp://localhost:59823?service=phasespace_mocap'

    mocap_cli = RRN.ConnectService(url)

    sensor_data_srv = mocap_cli.fiducials_sensor_data.Connect(-1)

    packet_num=0
    packet_num_total=1000
    st = time.time()
    all_stamps=[]
    while True:
        try:
            data = sensor_data_srv.ReceivePacketWait()
            all_stamps.append(float(data.sensor_data.ts[0]['seconds'])+data.sensor_data.ts[0]['nanoseconds']*1e-9)
            packet_num+=1
            if packet_num>=packet_num_total:
                break
        except KeyboardInterrupt:
            break
    print("Frame No.:",data.sensor_data.seqno)
    print("Total markers:",len(data.fiducials.recognized_fiducials))
    for i in range(len(data.fiducials.recognized_fiducials)):
        print("ID:",data.fiducials.recognized_fiducials[i].fiducial_marker)
        print("position:",data.fiducials.recognized_fiducials[i].pose.pose.pose[0]['position'])
        print("orientation:",data.fiducials.recognized_fiducials[i].pose.pose.pose[0]['orientation'])
        print("condition:",data.fiducials.recognized_fiducials[i].confidence)
        print("===========================================")
    et = time.time()
    print("Ave FPS:",packet_num_total/(et-st))
    # print(all_stamps)
    print((all_stamps[-1]-all_stamps[0])/packet_num_total)

if __name__=='__main__':
    main()
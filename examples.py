if __name__ == "__main__":
    from LLV3.Lidar_Lite_V3 import Lidar

    if __name__ == "__main__":
        my_lidar = Lidar()
        
        distance = my_lidar.get_distance()
        print(distance)
        distance = my_lidar.get_distance(bias_correction=False)
        print(distance)

        my_lidar.burst_measurement()
        my_lidar.burst_measurement(count=10, delay=0.1)
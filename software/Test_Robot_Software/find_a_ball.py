import image_processor
import camera
import cv2
import time
import api

def main_loop():
    debug = True
    
    cam = camera.RealsenseCamera(exposure = 100)    
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()

    start = time.time() 
    fps = 0
    frame = 0
    frame_cnt = 0
    
    scaled_pos_x = 0.0
    scaled_pos_y = 0.0

    try:
        while True:
            
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processed_data = processor.process_frame(aligned_depth=False)
            # largest = max(processed_data.balls, key = lambda ball: ball.size, default=None)
            largest = processed_data.balls[-1]
            if(largest):
                cv2.circle(processed_data.debug_frame,(largest.x, largest.y), 20, (255, 0, 255), -1)
                                
                # scaled_pos_x = (largest.x - processed_data.debug_frame.shape[1]/2) / (processed_data.debug_frame.shape[1]/2) 
                # scaled_pos_y = (processed_data.debug_frame.shape[0]/2 - largest.y) / (processed_data.debug_frame.shape[0]/2)                

                scaled_pos_x = (largest.x - cam.rgb_width/2) / (cam.rgb_width/2) 
                scaled_pos_y = (cam.rgb_height/2 - largest.y) / (cam.rgb_height/2)                
                              

                if (scaled_pos_y < 0.0):
                    speed_y = 0.0                
                
                else:                    
                    speed_y = scaled_pos_y  * 60 
                    
                
                if (speed_y > 30.0):
                    speed_y = 30.0
                else:
                    pass
                
                speed_x = scaled_pos_x * 16   # drive
                

                if (speed_x > 8.0):
                    speed_x = 8.0
                elif (speed_x < -8.0):
                    speed_x = -8.0
                else:
                    pass

                # print(f"Frame Size: ({processed_data.debug_frame.shape[1]}, {processed_data.debug_frame.shape[0]})")
                print(f"Frame Size: ({cam.rgb_width}, {cam.rgb_height})")
                print(f"scaled position: ({scaled_pos_x},{scaled_pos_y})")
                print(f"speed_x: {speed_x}")     
                print(f"speed_y: {speed_y}")     
                      
                api.RobotMovement.move(speed_x, speed_y, -speed_x*1.5)

            else:
                pass

            # print(largest)           

           # This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects

            frame_cnt +=1

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                print("ball_count: {}".format(len(processed_data.balls)))

                #if (frame_cnt > 1000):
                #    break

            if debug:
                debug_frame = processed_data.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    api.move(0.5, 0.5, 0.5)
                    break
    except KeyboardInterrupt:
        api.RobotMovement.move(0.5, 0.5, 0.5)
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        api.RobotMovement.close()
main_loop()

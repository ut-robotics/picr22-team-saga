import image_processor
import camera
import cv2
import time

def main():
    debug = True   
    
    cam = camera.RealsenseCamera(exposure = 100)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()    

    start = time.time() 
    fps = 0
    frame = 0
    frame_cnt = 0
    try:
        while True:
            
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processed_data = processor.process_frame(aligned_depth=False)
            # largest = max(processed_data.balls, key = lambda ball: ball.size, default=None)
            largest = processed_data.balls[-1]
            if(largest):
                cv2.circle(processed_data.debug_frame,(largest.x, largest.y), 20, (255, 0, 255), -1)
                
            print(largest)

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
                    break
    except KeyboardInterrupt:
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()

if __name__ == "__main__":
    main()

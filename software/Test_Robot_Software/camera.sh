ffmpeg -i /dev/video2 -vcodec libx264 -x264-params nal-hrd=cbr:force-cfr=1:keyint=250 -preset medium -profile high -pix_fmt yuv420p -tune zerolatency -b:v 2000K -minrate 2000K -maxrate 2000K -bufsize 4000k -f mpegts udp://192.168.3.33:49411 & ffmpeg -i /dev/video4 -vcodec libx264 -x264-params nal-hrd=cbr:force-cfr=1:keyint=250 -preset medium -profile high -pix_fmt yuv420p -tune zerolatency -b:v 2000K -minrate 2000K -maxrate 2000K -bufsize 4000k -f mpegts udp://192.168.3.33:49410

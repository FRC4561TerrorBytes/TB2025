package frc.robot.util;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.VideoSource;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoWriter;

public class VisionRecorder {
  private CvSink cvSink;
  private VideoWriter writer;

  public VisionRecorder(VideoSource camera, int fps) {
    cvSink = new CvSink("opencv_sink");
    cvSink.setSource(camera);

    // Path on USB stick plugged into RIO
    String filename = "/u/limelight.avi";

    writer =
        new VideoWriter(
            filename,
            VideoWriter.fourcc('M', 'J', 'P', 'G'),
            fps,
            new org.opencv.core.Size(320, 240));
  }

  public void update() {
    Mat frame = new Mat();
    if (cvSink.grabFrame(frame) > 0) {
      Mat resized = new Mat();
      Imgproc.resize(frame, resized, new Size(320, 240));

      // writing the resized image to the video file
      writer.write(resized);

      // release means done processing image and free up memory
      resized.release();
    }
    // release means free up memory
    frame.release();
  }

  public void close() {
    // done with this so free up memory by closing it
    // note can corrupt the .avi file if this is not ran
    writer.release();
  }
}

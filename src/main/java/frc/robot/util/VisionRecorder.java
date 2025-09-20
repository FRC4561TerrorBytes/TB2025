package frc.robot.util;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoWriter;

public class VisionRecorder {
  private CvSink cvSink;
  private VideoWriter writer;
  private String filename;

  public VisionRecorder(VideoSource camera, int fps, int cameraNumber) {
    cvSink = new CvSink("opencv_sink");
    cvSink.setSource(camera);

    // Match Number > 0 means that robot is at comp

    // Path on USB stick plugged into RIO
    if (DriverStation.getMatchNumber() > 0) {
      if (cameraNumber == 1) {
        filename =
            "/u/limelightRecordings/FrontLeft/"
                + DriverStation.getEventName()
                + "_"
                + DriverStation.getMatchType()
                + "-"
                + DriverStation.getMatchNumber()
                + ".avi";
      } else if (cameraNumber == 2) {
        filename =
            "/u/limelightRecordings/BackRight/"
                + DriverStation.getEventName()
                + "_"
                + DriverStation.getMatchType()
                + "-"
                + DriverStation.getMatchNumber()
                + ".avi";
      }
    } else {
      if (cameraNumber == 1) {
        filename =
            "u/limelightRecordings/FrontLeft/"
                + "recording_"
                + getFileNumber(cameraNumber)
                + ".avi";
      } else if (cameraNumber == 2) {
        filename =
            "u/limelightRecordings/BackRight/"
                + "recording_"
                + getFileNumber(cameraNumber)
                + ".avi";
      }
    }

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

  public int getFileNumber(int cameraNumber) {
    Path path = Paths.get("u/limelightRecordings");
    int counter = 0;
    if (cameraNumber == 1) {
      path = Paths.get("u/limelightRecordings/FrontLeft/recordingCounter.txt");
    } else if (cameraNumber == 2) {
      path = Paths.get("u/limelightRecordings/BackRight/recordingCounter.txt");
    }
    try {
      if (Files.exists(path)) {
        counter = Integer.parseInt(Files.readString(path).trim());
      }
      counter++;
      Files.writeString(path, Integer.toString(counter));
    } catch (IOException e) {
      e.printStackTrace();
    }
    return counter;
  }
}

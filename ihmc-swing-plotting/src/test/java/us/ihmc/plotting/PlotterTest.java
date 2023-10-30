package us.ihmc.plotting;

import static us.ihmc.robotics.Assert.assertEquals;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.plotting.PlotterPoint2d;
import us.ihmc.graphicsDescription.plotting.artifact.LineArtifact;
import us.ihmc.graphicsDescription.plotting.frames.MetersReferenceFrame;
import us.ihmc.graphicsDescription.plotting.frames.PixelsReferenceFrame;
import us.ihmc.graphicsDescription.plotting.frames.PlotterFrameSpace;
import us.ihmc.graphicsDescription.plotting.frames.PlotterSpaceConverter;

@Tag("gui")
public class PlotterTest
{
   @Test// timeout = 300000
   public void testPlotter()
   {
      Plotter plotter = new Plotter();

      plotter.addArtifact(new LineArtifact("01", new Point2D(0, 0), new Point2D(1, 1)));
      plotter.addArtifact(new LineArtifact("02", new Point2D(1, 1), new Point2D(2, 0)));
      plotter.addArtifact(new LineArtifact("03", new Point2D(2, 0), new Point2D(3, 1)));

      plotter.showInNewWindow();
      
      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         ThreadTools.sleepForever();
      }
   }

   @SuppressWarnings("serial")
   @Test// timeout = 30000
   public void testPlotterReferenceFrames()
   {
      PlotterSpaceConverter spaceConverter = new PlotterSpaceConverter()
      {
         private Vector2D scaleVector = new Vector2D();

         @Override
         public Vector2D getConversionToSpace(PlotterFrameSpace plotterFrameType)
         {
            if (plotterFrameType == PlotterFrameSpace.METERS)
            {
               scaleVector.set(0.1, 0.2);
            }
            else
            {
               scaleVector.set(10.0, 5.0);
            }
            return scaleVector;
         }
      };

      PixelsReferenceFrame pixelsFrame = new PixelsReferenceFrame("pixelsFrame", ReferenceFrame.getWorldFrame(), spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };

      PixelsReferenceFrame screenFrame = new PixelsReferenceFrame("screenFrame", pixelsFrame, spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.getTranslation().set(50.0, 100.0, 0.0);
            transformToParent.appendYawRotation(Math.PI);
         }
      };

      MetersReferenceFrame metersFrame = new MetersReferenceFrame("metersFrame", ReferenceFrame.getWorldFrame(), spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };

      pixelsFrame.update();
      metersFrame.update();
      screenFrame.update();

      PlotterPoint2d point = new PlotterPoint2d(metersFrame);

      point.set(1.0, 5.0);

      System.out.println(point);
      EuclidCoreTestTools.assertEquals("Point not equal", new Point2D(1.0, 5.0), point, 1e-7);

      point.changeFrame(pixelsFrame);

      System.out.println(point);
      EuclidCoreTestTools.assertEquals("Point not equal", new Point2D(10.0, 25.0), point, 1e-7);

      point.changeFrame(screenFrame);

      System.out.println(point);
      EuclidCoreTestTools.assertEquals("Point not equal", new Point2D(40.0, 75.0), point, 1e-7);

      point.changeFrame(metersFrame);

      System.out.println(point);
      EuclidCoreTestTools.assertEquals("Point not equal", new Point2D(1.0, 5.0), point, 1e-7);
   }

   @Test// timeout = 30000
   public void testBuildAPlotterAndCallSomeStuff()
   {
      Plotter plotter = new Plotter();
      plotter.setPreferredSize(800, 600);

      plotter.setScale(10.0, 10.0);
      plotter.setShowLabels(true);

      //      plotter.addArtifact(new LineArtifact("01", new Point2d(0, 0), new Point2d(1, 1)));
      //      plotter.addArtifact(new LineArtifact("02", new Point2d(1, 1), new Point2d(2, 0)));
      //      plotter.addArtifact(new LineArtifact("03", new Point2d(2, 0), new Point2d(3, 1)));

      plotter.showInNewWindow();

      plotter.setScale(40.0, 20.0);
      plotter.setFocusPointX(-5.0);
      plotter.setFocusPointY(10.0);

      assertEquals("focus point x not correct", -5.0, plotter.getFocusPointX(), 1e-7);
      assertEquals("focus point y not correct", 10.0, plotter.getFocusPointY(), 1e-7);
   }
}

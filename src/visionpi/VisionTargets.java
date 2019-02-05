/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 *//*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author RussS9
 */
package visionpi;

import com.sun.org.apache.bcel.internal.generic.TABLESWITCH;
import java.awt.List;
import java.util.ArrayList;
import org.opencv.core.Point;
import java.util.Comparator;
import java.util.Collections;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;  
import org.opencv.imgproc.Imgproc;
import static visionpi.VisionPi.vTargs;



public class VisionTargets{
    
    private ArrayList<TargetRectangle>   targetRectangles;
    public ArrayList<TargetLine>   targetLines;
    static final int PIX_WIDTH = 640;
    static final int PIX_HEIGHT = 480;
  
    static final double MIN_ASPECT_RATIO = 1.4; //This needs to be considerably lower to handle broken or ends of lines missing, tune again after dialed in height and angles
    static final double MAX_ASPECT_RATIO = 20.0;//expect 9:1 - Valid tape is usually 1.6 to 4.0
    
   
    private boolean lineProcessingReady = false;
    double tiltAngle = 14.0;
    double tiltThreshold = 5.0;
    Point centerPoint;   
    boolean leftTilt;
    private TargetLine bestTargetLine;
    
    public VisionTargets()
    {
        targetRectangles = new ArrayList<>();   //holds target object for each found target half
        targetLines = new ArrayList<>();        //holds target object for line segment
        bestTargetLine = new TargetLine(new Point(-1,-1), new Point (-1,-1),-1,-1);
    }
    
    class stringLengthComparitor implements Comparator<String>
    {
        @Override
        public int compare(String s1, String s2)
        {
            int len1 = s1.length();
            int len2 = s2.length();
            if(len1 == len2)
            {
                return 0;
            }
            if(len1 < len2)
            {
                return 1;
            }
            return -1;
        }
    }
    
    public void addLineSegment(Point endPoint1, Point endPoint2, double intensityLeftOfLine, double intensityRightOfLine)
    {
        TargetLine target = new TargetLine(endPoint1, endPoint2, intensityLeftOfLine, intensityRightOfLine);
        target.centerPoint.x  = (endPoint1.x + endPoint2.x)/2;
        target.centerPoint.y  = (endPoint1.y + endPoint2.y)/2;
        targetLines.add(target);
        lineProcessingReady = false;
    }
   
    public void addRectangle(Point centerPoint, double tiltAngle)
    {
        TargetRectangle target = new TargetRectangle(centerPoint, tiltAngle);
        targetRectangles.add(target);
    }
    public int getLineCount()
    {
        return targetLines.size();
    }
 
    public int getRectangleCount()
    {
        return targetRectangles.size();
    }
    
    public void reset()
    {
        targetRectangles.clear();
        targetLines.clear();
        lineProcessingReady = false;
    }
    public Boolean foundValidTargetRectanglePair()
    {
        if(targetRectangles.size() >= 2)
        {
//            Collections.sort(targets, (a, b) -> b.compareTo(a));
  
//            targets.sort(centerPoint.x);
            return true;
        }
        return false;
    }
    public Point getBestTargetPoint()
    {
        Point pRet = new Point(-1,-1);
        if(foundValidTargetRectanglePair())
        {
            pRet.x = (targetRectangles.get(0).centerPoint.x + targetRectangles.get(1).centerPoint.x)/2;
            pRet.y = (targetRectangles.get(0).centerPoint.y + targetRectangles.get(1).centerPoint.y)/2;
        }
        return pRet;
    }
   
    public boolean foundLineTarget()
    {
        if(!lineProcessingReady)
        {
            repairSegments(); 
            groupSegmentPairs();
            lineProcessingReady = true;
        }
        return false;
    }
    public Point getBestTargetLine()
    {
        if(!lineProcessingReady)
        {
            repairSegments(); 
            groupSegmentPairs();
            lineProcessingReady = true;
        }
        return getBestTargetLineInfo();
    }
    
    public void repairSegments()//if segment endpoints are close and angles are close, replace segments with one long segment
    {
       
        for(int i = 0; i < targetLines.size()-1;i++)
        {
            for(int j = i+1; j<targetLines.size();j++)
            {
                TargetLine line1 = targetLines.get(i);    
                TargetLine line2 = targetLines.get(j);    
               
                double endPointDistanceTolerance = 10.0;//how far can endpoints be before we combine segments ? distance in pixels
                double lineAngleTolerance = 10.0;//how close does angle have to be before we combine segments ? angle in degrees
                if(Math.abs(line1.angleContinuous_0_TO_180()- line2.angleContinuous_0_TO_180()) < lineAngleTolerance)
                {
                    //With 2 lines, there are 4 cases to consider 
                    if(distanceBetweenPoints(line1.endPoint1, line2.endPoint1) < endPointDistanceTolerance  )
                    {
                        line1.endPoint1 = line2.endPoint2;
                        //set the new center point to the center of line 1
                        targetLines.get(i).centerPoint = new Point((line1.endPoint1.x + line1.endPoint2.x)/2,(line1.endPoint1.y + line1.endPoint2.y)/2);
                        targetLines.remove(j);
                    }
                    else
                    {
                        if(distanceBetweenPoints(line1.endPoint1, line2.endPoint2) < endPointDistanceTolerance )
                        {
                            line1.endPoint1 = line2.endPoint1;
                            //set the new center point to the center of line 1
                            targetLines.get(i).centerPoint = new Point((line1.endPoint1.x + line1.endPoint2.x)/2,(line1.endPoint1.y + line1.endPoint2.y)/2);
                            targetLines.remove(j);
                        }
                        else
                        {
                            if(distanceBetweenPoints(line1.endPoint2, line2.endPoint1) < endPointDistanceTolerance  )
                            {
                                line1.endPoint2 = line2.endPoint2;
                                //set the new center point to the center of line 1
                                targetLines.get(i).centerPoint = new Point((line1.endPoint1.x + line1.endPoint2.x)/2,(line1.endPoint1.y + line1.endPoint2.y)/2);
                                targetLines.remove(j);
                            }
                            else
                            {
                                if(distanceBetweenPoints(line1.endPoint2, line2.endPoint2) < endPointDistanceTolerance  )
                                {
                                    line1.endPoint2 = line2.endPoint1;
                                    //set the new center point to the center of line 1
                                    targetLines.get(i).centerPoint = new Point((line1.endPoint1.x + line1.endPoint2.x)/2,(line1.endPoint1.y + line1.endPoint2.y)/2);
                                    targetLines.remove(j);
                                }
                            }
                        }
                    }
                }
            }           
        }
    }
    private void groupSegmentPairs()//find pairs of segments that could be sides of 2" x 18" tape
    {
        
    }
    public Point getTargetLineEndPoint(int index, boolean firstPointPlease)
    {
        Point pRet = new Point(-1,-1);
        if(targetLines.size() > index)
        {
            TargetLine line = targetLines.get(index);
            if(firstPointPlease)
            {
                return line.endPoint1;
            }
            return line.endPoint2;
        }
        return pRet;
    }
    public double getTargetLineLength(int index)
    {
        return Math.abs(distanceBetweenPoints(targetLines.get(index).endPoint1, targetLines.get(index).endPoint2));
    }
    private double distanceBetweenPoints(Point p1, Point p2)
    {
        double deltaX = p2.x - p1.x;
        double deltaY = p2.y - p1.y;
        return Math.sqrt(deltaX*deltaX + deltaY*deltaY);
    }
    private Point getBestTargetLineInfo()//return the head and tail points 
    {
        Point pRet = new Point(-1,-1);
        return pRet;
    }
    public boolean isLinePairTarget(Mat matDrawOn, Mat matTestOn, int lineIndex1, int lineIndex2, double expectedLineLengthMinPixels, double expectedLineLengthMaxPixels)
    {
        //min max line length
        //90degree H angle  +/- tolerance
        //min max aspect ratio
        // dark to light : light to dark
        //
        
        //make sure both lines exist 
        if(targetLines.size() <= lineIndex1 || targetLines.size() < lineIndex2)
        {         
            return false;
        }
        // get the two lines and the line crossing the center points thus forming an H
        TargetLine line1 = targetLines.get(lineIndex1);
        TargetLine line2 = targetLines.get(lineIndex2);
        TargetLine lineCross = new TargetLine(line1.centerPoint,line2.centerPoint,-1,-1);
        
        //filter line lengths
        if( line1.length() < expectedLineLengthMinPixels || 
            line2.length() < expectedLineLengthMinPixels || 
            line1.length() > expectedLineLengthMaxPixels ||
            line2.length() > expectedLineLengthMaxPixels )
        {
            return false;
        }


        //Now that we have the three lines, check to see if it forms an H
        double angle1 = targetLines.get(lineIndex1).angleContinuous_0_TO_180();
        double angle2 = targetLines.get(lineIndex2).angleContinuous_0_TO_180();
        double angleCross = lineCross.angleContinuous_0_TO_180();
        
        double angleH_tolerance = 75; //need to keep this over 60 degrees
                                      // to pass skinny H's with bad lines caused by various distortions , degrees
   
        //Filter each angle made by the crossing line and the other two lines
        // Angles are always positive (0 to 180)
        // There are many ways to do this. One way is to subtract smaller from the larger and compare to 90
        // This is not the same as ABS function
        double angleDiff1 = angle1 - angleCross;
        double angleDiff2 = angle2 - angleCross;
        
        if(angle1 < angleCross)
        {
            angleDiff1 = angleCross - angle1;
        }
        if(angle2 < angleCross)
        {
            angleDiff2 = angleCross - angle2;
        }
        
        if( Math.abs(90 - angleDiff1) > angleH_tolerance ||
            Math.abs(90 - angleDiff2) > angleH_tolerance)
        {
            return false; //the crossing line of the H is not close enough to 90 degrees
        }
      
        //filter lines that are not parallel
        double parallelLineAngleTolerance = 10;
        if(Math.abs(angle1-angle2) > parallelLineAngleTolerance)
        {
            return false;
        }
        
        //Filter the aspect ratio of the H         
        //Target line is a 2" x 18" peice of tape i.e. acpect ratio 9:1
        //
        double ratio1 = (targetLines.get(lineIndex1).length() / lineCross.length());
        double ratio2 = (targetLines.get(lineIndex2).length() / lineCross.length());
        if( (ratio1 < MIN_ASPECT_RATIO) || 
            (ratio2 < MIN_ASPECT_RATIO) ||
            (ratio1 > MAX_ASPECT_RATIO) ||
            (ratio2 > MAX_ASPECT_RATIO))
        {
            return false;
        }
            
        //***** Keep this filter last so we only put text if the line pair is accepted as valid ***********
        //Filter to make sure we are considering a thick white line, not stray lines with dark pixels between them
        // We only want to insure dark:light for left line and light:dark for the right line                             
        //Determine which line is left of the other
        if(line1.centerPoint.x < line2.centerPoint.x)
        {
           //line 1 is left most
            if(line1.intensityLeftOfLine > line1.intensityRightOfLine ||
               line2.intensityLeftOfLine < line2.intensityRightOfLine)
            {
               return false;
            }
            Imgproc.putText(matDrawOn, String.format("%.0f %.0f %.0f %.0f",     line1.intensityLeftOfLine,
                                                                                line1.intensityRightOfLine,
                                                                                line2.intensityLeftOfLine,
                                                                                line2.intensityRightOfLine), 
                                                                                line1.endPoint1, 
                                                                                Core.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(0,0,255));
        }
        else
        {
           //line 2 is left most
            if(line2.intensityLeftOfLine > line2.intensityRightOfLine ||
               line1.intensityLeftOfLine < line1.intensityRightOfLine)
            {
               return false;
            }
            Imgproc.putText(matDrawOn, String.format("%.0f %.0f %.0f %.0f",     line2.intensityLeftOfLine,
                                                                                line2.intensityRightOfLine,
                                                                                line1.intensityLeftOfLine,
                                                                                line1.intensityRightOfLine), 
                                                                                line1.endPoint1, 
                                                                                Core.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(0,0,255));
        }
        //***** Keep the above filter last so we only put text if the line pair is accepted as valid ***********
        //System.out.println(String.format("len %.1f %.1f %.1f", lineCross.length(),ratio1 , ratio2));
        return true; // valid target pair
    }
    
    public Point lineLineIntersection(Point A, Point B, Point C, Point D) 
    { 
        // Line AB represented as a1x + b1y = c1 
        double a1 = B.y - A.y; 
        double b1 = A.x - B.x; 
        double c1 = a1*(A.x) + b1*(A.y); 
       
        // Line CD represented as a2x + b2y = c2 
        double a2 = D.y - C.y; 
        double b2 = C.x - D.x; 
        double c2 = a2*(C.x)+ b2*(C.y); 
       
        double determinant = a1*b2 - a2*b1 +0.000001;//avoid div by zero, works for whole doubles 
       
        if (determinant == 0) 
        { 
            // The lines are parallel. This is simplified 
            // by returning a pair of FLT_MAX 
            return new Point(620, 460); 
        } 
        else
        { 
            double x = (b2*c1 - b1*c2)/determinant; 
            double y = (a1*c2 - a2*c1)/determinant; 
            return new Point(x, y); 
        } 
    } 
 
    public double getHLineAngle(int lineIndex1, int lineIndex2)
    {
        //line pair is a target when
        //  adding a line connecting center points forms an H shape
        //  ratio of average line length and distance between lines is withing range
        //  vanishing point is in a reasonable place
        //make sure both lines exist 
        if(targetLines.size() <= lineIndex1 || targetLines.size() < lineIndex2)
        {         
            return -1;
        }

        //copy the 4 x and 4 y values to make things easier to read
        double line1P1x = targetLines.get(lineIndex1).endPoint1.x;
        double line1P2x = targetLines.get(lineIndex1).endPoint2.x;
        double line1P1y = targetLines.get(lineIndex1).endPoint1.y;
        double line1P2y = targetLines.get(lineIndex1).endPoint2.y;
        
        double line2P1x = targetLines.get(lineIndex2).endPoint1.x;
        double line2P2x = targetLines.get(lineIndex2).endPoint2.x;
        double line2P1y = targetLines.get(lineIndex2).endPoint1.y;
        double line2P2y = targetLines.get(lineIndex2).endPoint2.y;

        //determine center point of each line
        double xAvg1stLine = (line1P1x + line1P2x) / 2;
        double yAvg1stLine = (line1P1y + line1P2y) / 2;
        double xAvg2ndLine = (line2P1x + line2P2x) / 2;
        double yAvg2ndLine = (line2P1y + line2P2y) / 2;

        TargetLine lineH = new TargetLine(new Point(xAvg1stLine, yAvg1stLine), new Point(xAvg2ndLine, yAvg2ndLine),-1,-1);
        return lineH.angleContinuous_0_TO_180();
    }

    public TargetLine getHLine(int lineIndex1, int lineIndex2)
    {
        //line pair is a target when
        //  adding a line connecting center points forms an H shape
        //  ratio of average line length and distance between lines is withing range
        //  vanishing point is in a reasonable place
        //make sure both lines exist 
        if(targetLines.size() <= lineIndex1 || targetLines.size() < lineIndex2)
        {         
            return new TargetLine(new Point(-1,-1), new Point(-1,-1),-1,-1);
        }
        TargetLine lineH = new TargetLine(targetLines.get(lineIndex1).centerPoint, targetLines.get(lineIndex2).centerPoint,-1,-1);
        return lineH;
    }
    
    public Point getStartingPointOfLinePair(int lineIndex1, int lineIndex2)
    {
        //This function returns the point between the lowest two ends of the lines 
        Point pRet = new Point(-1,-1);
        //make sure both lines exist 
        if(targetLines.size() > lineIndex1 && targetLines.size() > lineIndex2)
        {
           //copy the 4 x and 4 y values to make things easier to read
           double x1_1 = targetLines.get(lineIndex1).endPoint1.x;
           double x1_2 = targetLines.get(lineIndex1).endPoint2.x;
           double x2_1 = targetLines.get(lineIndex2).endPoint1.x;
           double x2_2 = targetLines.get(lineIndex2).endPoint2.x;
           double y1_1 = targetLines.get(lineIndex1).endPoint1.y;
           double y1_2 = targetLines.get(lineIndex1).endPoint2.y;
           double y2_1 = targetLines.get(lineIndex2).endPoint1.y;
           double y2_2 = targetLines.get(lineIndex2).endPoint2.y;

           //identify the highest y value for each line and save the x as well
           //positive y is down
           double lowestX1 = x1_1;
           double lowestY1 = y1_1;
           if(y1_2 > y1_1)
           {
                lowestX1 = x1_2;
                lowestY1 = y1_2;
           }

           double lowestX2 = x2_1;
           double lowestY2 = y2_1;
           if(y2_2 > y2_1)
           {
                lowestX2 = x2_2;
                lowestY2 = y2_2;
           }
           //the average is the center point
           pRet.x = (lowestX1 + lowestX2)/2;
           pRet.y = (lowestY1 + lowestY2)/2;
        }
        return pRet;
    }

    public Point getEndingPointOfLinePair(int lineIndex1, int lineIndex2)
    {
        //This function returns the point between the highest two ends of the lines 
        Point pRet = new Point(-1,-1);
        //make sure both lines exist 
        if(targetLines.size() > lineIndex1 && targetLines.size() > lineIndex2)
        {
           //copy the 4 x and 4 y values to make things easier to read
           double x1_1 = targetLines.get(lineIndex1).endPoint1.x;
           double x1_2 = targetLines.get(lineIndex1).endPoint2.x;
           double x2_1 = targetLines.get(lineIndex2).endPoint1.x;
           double x2_2 = targetLines.get(lineIndex2).endPoint2.x;
           double y1_1 = targetLines.get(lineIndex1).endPoint1.y;
           double y1_2 = targetLines.get(lineIndex1).endPoint2.y;
           double y2_1 = targetLines.get(lineIndex2).endPoint1.y;
           double y2_2 = targetLines.get(lineIndex2).endPoint2.y;

           //identify the lowest y value for each line and save the x as well
           //positive y is down
           double highestX1 = x1_1;
           double highestY1 = y1_1;
           if(y1_2 < y1_1)
           {
                highestX1 = x1_2;
                highestY1 = y1_2;
           }

           double highestX2 = x2_1;
           double highestY2 = y2_1;
           if(y2_2 < y2_1)
           {
                highestX2 = x2_2;
                highestY2 = y2_2;
           }
           //the average is the center point
           pRet.x = (highestX1 + highestX2)/2;
           pRet.y = (highestY1 + highestY2)/2;
        }
        return pRet;
    }

    private class TargetRectangle
    {
        Point centerPoint;
        double tiltAngle;
        public TargetRectangle(Point centerPoint_in, double tiltAngle_in)
        {
            centerPoint = new Point(centerPoint_in.x, centerPoint_in.y);
            tiltAngle = tiltAngle_in;
        }
        public Boolean isLeft()
        {
            return tiltAngle > 0;
        }
        public Boolean isRight()
        {
            return tiltAngle <= 0;//angles are always near 14 degrees but consider = for compleatness
        }
    }
    public class TargetLine
    {
        Point endPoint1;
        Point endPoint2;
        Point centerPoint;
        double intensityLeftOfLine;
        double intensityRightOfLine;
        public TargetLine(Point endPoint1_in, Point endPoint2_in, double intensityLeft_in, double intensityRight_in)
        {
            endPoint1 = new Point(endPoint1_in.x, endPoint1_in.y);
            endPoint2 = new Point(endPoint2_in.x, endPoint2_in.y);
            centerPoint = new Point((endPoint1_in.x + endPoint2_in.x) / 2,(endPoint1_in.y + endPoint2_in.y) / 2);
            intensityLeftOfLine=intensityLeft_in;
            intensityRightOfLine=intensityRight_in;
        }
        public double length()
        {
            return distanceBetweenPoints(endPoint1, endPoint2);
        }
        public double angle()//returns +/- 90 degrees
        {
           double m =  (endPoint1.y - endPoint2.y)/(endPoint1.x - endPoint2.x + 0.0000001);//add a bit to avoid div by 0 - can do this when dealing with whole doubles or ints
           double angle = -Math.toDegrees(Math.atan(m));    //calculate angle of line
           return angle;
        }
        public double angleContinuous_0_TO_180()//returns 0 to 180 degrees
        {
           double m =  (endPoint1.y - endPoint2.y)/(endPoint1.x - endPoint2.x + 0.0000001);//add a bit to avoid div by 0 - can do this when dealing with whole doubles or ints
           double angle = -Math.toDegrees(Math.atan(m));    //calculate angle of line
           if(angle<0)
           {
               angle += 180;
           }
           return angle;
        }
        public Boolean isLongSegment()
        {
            return false;
        }
        public Boolean isShortSegment()
        {
            return false;
        }
    }
}

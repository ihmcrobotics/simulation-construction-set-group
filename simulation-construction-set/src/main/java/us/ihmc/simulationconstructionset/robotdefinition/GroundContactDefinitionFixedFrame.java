package us.ihmc.simulationconstructionset.robotdefinition;

import us.ihmc.euclid.tuple3D.Vector3D;

public class GroundContactDefinitionFixedFrame
{
   String name = null;

   Vector3D offset = null;

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public Vector3D getOffset()
   {
      return offset;
   }

   public void setOffset(Vector3D offset)
   {
      this.offset = offset;
   }

   public String getXMLRepresentation()
   {
      String returnString = "";
      returnString += "<GroundContactPoint>\n";
      returnString += "\t<Name>" + name + "</Name>\n";
      returnString += "\t<Offset>" + offset + "</Offset>\n";

      returnString += "</GroundContactPoint>\n";

      return returnString;
   }

}

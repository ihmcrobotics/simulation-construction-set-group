plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("net.sf.trove4j:trove4j:3.0.3")
   api("org.jfree:jfreechart:1.0.19")
   api("org.jfree:jcommon:1.0.24")

   api("us.ihmc:euclid:0.20.0")
   api("us.ihmc:ihmc-commons:0.32.0")

   api("us.ihmc:ihmc-graphics-description:source")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.32.0")

   api("us.ihmc:ihmc-graphics-description-test:source")
}

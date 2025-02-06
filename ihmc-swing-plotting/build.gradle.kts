plugins {
   id("us.ihmc.ihmc-build")
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

   api("us.ihmc:euclid:0.22.3")
   api("us.ihmc:ihmc-commons:0.35.1")

   api("us.ihmc:ihmc-graphics-description:0.26.1")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.35.1")

   api("us.ihmc:ihmc-graphics-description-test:0.26.1")
}

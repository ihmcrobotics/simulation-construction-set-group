plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.22.2")

   api("us.ihmc:ihmc-graphics-description:source")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.34.0")
}

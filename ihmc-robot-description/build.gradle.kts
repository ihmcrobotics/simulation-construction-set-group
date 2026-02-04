plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.22.5")

   api("us.ihmc:ihmc-graphics-description:0.27.0")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.35.1")
}

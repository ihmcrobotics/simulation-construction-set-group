plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.gradle.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.22.4")

   api("us.ihmc:ihmc-graphics-description:0.26.2")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.35.1")
}

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
}

ihmc {
   group = "us.ihmc"
   version = "0.0"
   vcsUrl = "https://stash.ihmc.us/users/sbertrand/repos/reachabilitymap"
   openSource = false

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:atlas:source")
   api("us.ihmc:valkyrie:source")
   api("us.ihmc:ihmc-avatar-interfaces:source")
}

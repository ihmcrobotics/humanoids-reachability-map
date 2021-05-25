
plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-cd") version "1.20"
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.log-tools-plugin") version "0.6.1"
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
   api("us.ihmc:nadia-hardware-drivers:source")
   api("us.ihmc:ihmc-avatar-interfaces:source")
}

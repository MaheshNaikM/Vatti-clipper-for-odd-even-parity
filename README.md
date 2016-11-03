# Mumbai polygon clipper a ascendent of 

General polygon clipper version 2.32 an 2.33 ...edited to remove faults and bugs.
The bugs are licensed NOt the solutions.

The main changes are
Intersection sequencing is done to get  adjacent bundles at each rendering step
In the triangle code Murta generated the strips(left and right arms proprly)but goofed up when the arnms are of uneaual length.
In triangle processing the duplicate elimination step at EMX was in error. 
 It was found that removing all duplicate detection code to output contour processing is a simpler solution.
 So all code was edited to remove these statements from the rendering commands.
 the huge monolith was factored was easier manipulations and profiling.
 
  The resulting code works only ofr odd/even parity.But is as fast as the fastest vatti clippers.
 
 Use and enjoy 
 

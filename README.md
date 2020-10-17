# Mumbai polygon clipper a ascendent of 

General polygon clipper version 2.32 an 2.33 ...edited to remove faults and bugs.
The bugs are licensed NOt the solutions.

The main changes are
Intersection sequencing is done to get  adjacent bundles at each non horizontal rendering step
In the triangle code Murta generated the strips(left and right arms properly)but goofed up when the arms are of uneaual length.
In triangle processing the duplicate elimination step at EMX was in error. 
 It was found that removing all duplicate detection code to output contour processing is a simpler solution.
 So all code was edited to remove these statements from the rendering commands.
 the huge monolith was factored is  easier for  manipulations and profiling.
 
  The resulting code works only for odd/even parity.But is as fast as the fastest vatti clippers.
 In the delphi code memory leaks are there.  Removed in next version...mail me for the code before i  post it here.
 Use and enjoy 
 
   aNYONE INTERESTED in Murtas polygon clipping code ..for odd/even Contours shoudl contact Ignorant at maheshnaikDOTmumbaiATgmailDOTcom. You will get the current code in cpp.It is as fast as the fastest boolean clippers..totally phogat and a lot of bugs removed from Murtas original code.

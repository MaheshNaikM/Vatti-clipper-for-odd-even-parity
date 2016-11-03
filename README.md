# Mumbai polygon clipper a ascendent of 

General polygon clipper version 2.32 an 2.33 ...edited to remove faults and bugs.
The bugs are licensed NOt the solutions.

The main changes are
Intersection sequencing to use at intersection steps only adjacent bundles
In the triangle code Murta generated the strips(left and right arms proprly)but goofed up when the arnms are of uneaual length.
In triangle processing the duplicate elimination step at EMX was in error. 
 It was found that removing all duplicate detection code output contour processing is a simpler solution.so all code was edited to remove these statements from the rendering commands.
 the huge monolith was factored was easier manipulations and profiling.
 
 Use and enjoy 
 

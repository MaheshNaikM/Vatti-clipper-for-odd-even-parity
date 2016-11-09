

   The single file contain a rewritten general polygon clipper
   lot of code has been rewritten and bugs removed.
   For triangulation code the manchester code was doing the last
   part of creating strips wrong which lead to loss of covered area.
    This is a delphi pascal version of the attempted rectifications.
   Please note that the bugs and faults will require a license from the original 
   IPR  claimants of gpc.Get the full documentations on the algorithms from them.
     Lot of algorithsm within the clipper have been changed.The main change being 
     to use only adjacent intersecting edges within a beam while rendering.To spped up
     the code on large input the now classical bound generation and scan line generation 
     steps are used instead of the %&& Manchester code.
     
      As of now it is only for odd/even winding.It is  as fast as any other vatti clipper on the 
      net.
      
       For Murtas algorithm and notations contact of Dr. Alan murta is necesarry as the license
       manager does Not have any worthwile help material.Search the net for him.
   

function ReduceColorOrderIndex(ax)

   if ax.ColorOrderIndex==1
       ax.ColorOrderIndex=7;
   else
       ax.ColorOrderIndex=ax.ColorOrderIndex-1;
   end
end
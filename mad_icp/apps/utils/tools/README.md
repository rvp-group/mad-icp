
# Data association and registration tools
### Data association
`nn_search.py` allows computing the data association between either:
- a query point and a reference cloud;
- a query cloud and a reference cloud.

For each query point, the nearest-neighbor search returns the corresponding reference point with the estimated normal.
Notice that the pybind of the tree build method has `b_max` parameter set to `1e-5` by default (check `src/pybind/tools/pymadtree.cpp`). A MAD-tree built with such a parameter has exactly one leaf for each point of the original cloud, thus providing the most dense possible reference.
Indeed, the total matching error printed by running `nn_search.py` as it is is zero (because query and reference cloud are the same). Highering `b_max` in the tree building leads to a shallower tree with fewer leaves, thus making the reference less dense than the query cloud, consequently increasing the total matching error.
To run this module:
```bash
python3 nn_search.py
```
### Registration
`mad_registration.py` registers a query cloud on a reference cloud, providing the estimated transformation between them. This module also allows us to visualize the cloud alignment at the end of each iteration, and the data reassociations between the two.
You can run this module with the option `--viz` for toggling visualization.
```bash
python3 mad_registration.py --viz
```

<div align="center">
        <a href=""><img src="mad-registration.gif?raw=true"/></a>   
</div>
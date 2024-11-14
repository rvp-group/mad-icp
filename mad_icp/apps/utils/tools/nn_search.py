# Copyright 2024 R(obots) V(ision) and P(erception) group
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy as np

# binded vectors and madtree
from mad_icp.src.pybind.pyvector import VectorEigen3d
from mad_icp.src.pybind.pymadtree import MADtree
from mad_icp.apps.utils.tools.tools_utils import generate_four_walls_pointcloud

np.random.seed(42)

if __name__ == '__main__':
    # generate a random 4 walls + floor point cloud 3d
    cloud = generate_four_walls_pointcloud()
    print("single point nn")
    # single point search with mad tree
    qp = cloud[0, :] # query point
    tree = MADtree()
    tree.build(VectorEigen3d(cloud))
    # ref point and normal calculated by mad tree
    ref_point, ref_normal = tree.search(qp)
    print("query point {}".format(qp))
    print("ref point {} | ref normal {}".format(ref_point, ref_normal))
    print("error in matching {}".format(np.linalg.norm(ref_point-qp)))

    print(10*"=")
    # full cloud search with mad tree
    print("full cloud nn")
    ref_cloud = tree.searchCloud(VectorEigen3d(cloud))
    
    tot_matching_err = 0.0
    for (ref_point, ref_normal), query_point in zip(ref_cloud, cloud):
        tot_matching_err += np.linalg.norm(ref_point-query_point)
    
    print("error in matching {}".format(tot_matching_err))


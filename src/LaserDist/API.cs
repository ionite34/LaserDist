using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace LiDAR
{
    /// <summary>
    /// The class to interface with kRPC
    /// </summary>
    public partial class API
    {
        private static API instance;
        private static API Instance
        {
            get
            {
                if (instance == null)
                    instance = new API();
                return instance;
            }
        }

        /// <summary>
        /// Gets the pointcloud from the current instance.
        /// </summary>
        public IList<double> InstanceGetCloud(Part part)
        {
            IList<double> cloudData = new List<double>();

            var lidarModules = part.Modules.OfType<LiDARModule>();

            foreach (LiDARModule lidar in lidarModules)
            {
                foreach (var point in lidar.cloudData)
                {
                    cloudData.Add(point);
                }
                lidar.ClearCloudData();
            }
            return cloudData;
        }

        /// <summary>
        /// Gets the current pointcloud. Called by kRPC.
        /// </summary>
        public static IList<double> GetCloud(Part part)
        {
            return Instance.InstanceGetCloud(part);
        }
    }
}


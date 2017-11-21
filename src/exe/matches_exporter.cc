// COLMAP - Structure-from-Motion and Multi-View Stereo.
// Copyright (C) 2017  Johannes L. Schoenberger <jsch at inf.ethz.ch>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <QApplication>

#include "base/feature_matching.h"
#include "util/logging.h"
#include "util/option_manager.h"

using namespace colmap;

int main(int argc, char** argv) {
  InitializeGlog(argv);

  std::string import_path;
  std::string export_path;

  OptionManager options;
  options.AddDatabaseOptions();
  options.AddDefaultOption("import_path", &import_path);
  options.AddRequiredOption("export_path", &export_path);
  options.Parse(argc, argv);

  /// Read database
  // Copied from
  // IncrementalMapperController::LoadDatabase()
  std::cout << "Loading database" << std::endl;
  Database database(*options.database_path);

  // Load matches
  std::cout << "Loading all matches..." << std::cout;
  std::vector<image_pair_t> image_pair_ids;
  std::vector<TwoViewGeometry> two_view_geometries;
  database.ReadAllInlierMatches(&image_pair_ids, &two_view_geometries);
  const size_t num_pairs = image_pair_ids.size();
  // - image_pair_ids has all pairs of matched images
  // needs decomposing the pair idx into two image ids
  // - two_view_geometries has inlier_matches, a list of
  // all matching points idxs
  // I don't understand why all intermediate fields are empty...
  // no qvec or tvec, E,F,H...

  // Setup output file
  std::ofstream file;
  file.open(export_path+".txt");
  file << "# List of bearing vector correspondences for image pairs in dataset:" << std::endl;
  file << "#   IMAGE_NAME_1, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME" << std::endl;
  file << "#   IMAGE_NAME_2, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME" << std::endl;
  file << "#   NUM_MATCHES BEARING_VECTORS[] as (f_x,f_y,f_z,f'_x,f'_y,f'_z)" << std::endl;
  file << "# Number of pairs: " << num_pairs << std::endl;

  // For each 2-view geometry pair
  const size_t min_num_matches = 6;
  for (size_t i = 0; i < num_pairs; ++i) {
    /// Read TwoViewGeometry information
    TwoViewGeometry tvg = two_view_geometries[i];
    size_t num_matches = tvg.inlier_matches.size();
    if( num_matches < min_num_matches )
      // Not enough inliers
      continue;
    // Extract matching point indices in each image into 2 vectors
    std::vector<size_t> img_pt_idxs[2];
    img_pt_idxs[0].resize(num_matches);
    img_pt_idxs[1].resize(num_matches);
    for (size_t p = 0; p<num_matches; p++)
    {
      img_pt_idxs[0][p] = tvg.inlier_matches[p].point2D_idx1;
      img_pt_idxs[1][p] = tvg.inlier_matches[p].point2D_idx2;
    }

    /// Get pair image ids
    image_t image_ids[2];
    Database::PairIdToImagePair(image_pair_ids[i], &image_ids[0], &image_ids[1]);

    // Reserve space for bearing vectors
    std::vector<Eigen::Vector3d> bearing_vectors[2];
    bearing_vectors[0].resize(num_matches);
    bearing_vectors[1].resize(num_matches);
//    std::cout << "Compute matching bearing vectors for images "
//              << image_ids[0] << " and " << image_ids[1]
//              << std::cout;
    // Iterate on both pair images
    for (size_t c = 0; c < 2; c++)
    {
      // Read image to recover camera object
      Image img  = database.ReadImage(image_ids[c]);
      // Get camera corresponding to this image
      Camera cam = database.ReadCamera(img.CameraId());
      // Write pair metadata to file
      file << img.Name() << std::endl;

      // Read feature points in the image
      const std::vector<Eigen::Vector2d> img_points =
          FeatureKeypointsToPointsVector(
            database.ReadKeypoints(image_ids[c]));
      // Iterate on each image features
      for (size_t p = 0; p < num_matches; p++)
      {
        // Get point from list corresponding to current match
        size_t pt_idx = img_pt_idxs[c][p];
        Eigen::Vector2d img_pt = img_points[pt_idx];

        // Use Camera method to normalize keypoint coordinates
        Eigen::Vector3d nrm_pt_hom;
        nrm_pt_hom.head<2>() = cam.ImageToWorld(img_pt);
        nrm_pt_hom[2] = 1.0;
        bearing_vectors[c][p] = nrm_pt_hom / nrm_pt_hom.norm();
      }
    }
    // Write bearing vectors to file
    file << num_matches << " ";
    for (size_t p = 0; p < num_matches; p++)
    {
      for (size_t c = 0; c < 2; c++)
        file << bearing_vectors[c][p].transpose() << " ";
    }
    file << std::endl;
  }
  // Finished writing the output file, close it
  file.close();

  return EXIT_SUCCESS;
}

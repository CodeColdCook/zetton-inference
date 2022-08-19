#pragma once

#include <ros/ros.h>

#include "zetton_common/util/log.h"

namespace zetton {
namespace inference {
namespace tracker {
class AssociationType {
 public:
  AssociationType() = default;
  AssociationType(int id_, float score_, float bbox_match_dis_)
      : id(id_), score(score_), bbox_match_dis(bbox_match_dis_) {}

  int id;
  float score;
  float bbox_match_dis;
};

class AssociationVector {
 public:
  AssociationVector() = default;

  void add(const AssociationType &ass_object) {
    if (ass_vector.empty()) {
      ass_vector.push_back(ass_object);
    } else {
      for (auto iter = ass_vector.begin(); iter != ass_vector.end(); iter++) {
        if (ass_object.score < iter->score) {
          ass_vector.insert(iter, ass_object);
          return;
        }
      }
      ass_vector.push_back(ass_object);
    }
  }

  void reranking() {
    // less than 2 object
    if (ass_vector.empty() || ass_vector.size() == 1) {
      return;
    }

    // ratio test for reranking
    if (ass_vector[0].score > 0.7 * ass_vector[1].score &&
        ass_vector[0].bbox_match_dis > ass_vector[1].bbox_match_dis) {
      AssociationType ass_tmp = ass_vector[0];
      ass_vector[0] = ass_vector[1];
      ass_vector[1] = ass_tmp;
    }
  }

  void report() {
    AINFO_F("Data Association Report:");
    for (auto ass : ass_vector) {
      AINFO_F("id: {} | score: {}", ass.id, ass.score);
    }
    AINFO_F(" ");
  }

  std::vector<AssociationType> ass_vector;
};

// this function try to uniquify the match, it makes sure one detected object
// only matches one tracking object you can take this matching process as
// solving a graph problem the goal is to find the edge with minimum cost of a
// detected object and a tracking object, and meanwhile pruning all the other
// edge
void uniquify_detector_association_vectors_once(
    std::vector<AssociationVector> &detector_association_vectors,
    std::vector<AssociationVector> &tracker_association_vectors,
    const int detector_index);

void uniquify_detector_association_vectors(
    std::vector<AssociationVector> &detector_association_vectors,
    const int local_track_list_num);
} // namespace tracker
}  // namespace inference
}  // namespace zetton
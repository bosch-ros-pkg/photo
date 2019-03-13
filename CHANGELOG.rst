^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package photo
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2019-03-13)
------------------
* update rosdep keys for libgphoto2
* Contributors: root

1.0.1 (2019-03-13)
------------------
* Melodic devel (`#4 <https://github.com/bosch-ros-pkg/photo/issues/4>`_)
  * Make it compile on Ubuntu Bionic
  Changes from code-iai fork: https://github.com/code-iai/iai_photo
  * Fix delete on array
  * Fix set config for toggle
  A bool was implicitly casted to int through a void*
  * Fix whitespaces
  * Fix segmentation fault if no camera connected
  * Pass std::string by const reference to avoid copies
  * Sanitize a bit more
* Merge pull request `#3 <https://github.com/bosch-ros-pkg/photo/issues/3>`_ from proan/hydro-devel
  Added another dependency and removed superfluous comments.
* Added another dependency and removed superfluous comments.
* Merge pull request `#2 <https://github.com/bosch-ros-pkg/photo/issues/2>`_ from proan/hydro-devel
  Catkin-ized package for photo library.
* Add dependency on message and service generation being complete first.
* Catkin-ized package for photo. This package has been moved from the larger bosch_drivers repository.
* Initial commit
* Contributors: Philip Roan, Romain Reignier

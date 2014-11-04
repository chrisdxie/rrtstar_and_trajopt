
bin/example_standalone_rrtstar_double_integrator 600 true other_things/random_scene_generation/double_integrator/scene_2 12345 2 &
bin/example_standalone_rrtstar_double_integrator 600 true other_things/random_scene_generation/double_integrator/scene_5 78 5 &

cd other_things/My_BITStar_Impl/double_integrator_exact_connector/build/bin/

./bitstar 600 true 150 ../../../../random_scene_generation/double_integrator/scene_2 144
mv pics/600_seconds.pdf pics/600_seconds_scene_2.pdf
./bitstar 600 true 150 ../../../../random_scene_generation/double_integrator/scene_5 145
mv pics/600_seconds.pdf pics/600_seconds_scene_5.pdf


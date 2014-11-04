# Move on to BIT* + exact connector for scene 2
cd other_things/My_BITStar_Impl/double_integrator/build/bin/

for i in $(seq 11 13);
    do ./bitstar 600 $i 150 ../../../../random_scene_generation/double_integrator/scene_2 $i &
done

./bitstar 600 14 150 ../../../../random_scene_generation/double_integrator/scene_2 14

for i in $(seq 15 17);
    do ./bitstar 600 $i 150 ../../../../random_scene_generation/double_integrator/scene_2 $i &
done

./bitstar 600 18 150 ../../../../random_scene_generation/double_integrator/scene_2 18

for i in $(seq 19 21);
    do ./bitstar 600 $i 150 ../../../../random_scene_generation/double_integrator/scene_2 $i &
done

./bitstar 600 22 150 ../../../../random_scene_generation/double_integrator/scene_2 22

mv BITSTAR_double_integrator* statistics/scene_2/
mv pics/600_seconds.pdf pics/600_seconds_scene_2.pdf

# Move on to BIT* + exact connector for scene 5
for i in $(seq 11 13);
    do ./bitstar 600 $i 150 ../../../../random_scene_generation/double_integrator/scene_5 $i &
done

./bitstar 600 14 150 ../../../../random_scene_generation/double_integrator/scene_5 14

for i in $(seq 15 17);
    do ./bitstar 600 $i 150 ../../../../random_scene_generation/double_integrator/scene_5 $i &
done

./bitstar 600 18 150 ../../../../random_scene_generation/double_integrator/scene_5 18

for i in $(seq 19 21);
    do ./bitstar 600 $i 150 ../../../../random_scene_generation/double_integrator/scene_5 $i &
done

./bitstar 600 22 150 ../../../../random_scene_generation/double_integrator/scene_5 22

mv BITSTAR_double_integrator* statistics/scene_5
mv pics/600_seconds.pdf pics/600_seconds_scene_5.pdf


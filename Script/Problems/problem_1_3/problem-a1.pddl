(define (problem rcll-production-task)
(:domain rcll-production-steps)
(:objects
   r1 - robot
   p0 - product
   silver_base_p0 gray_cap_p0 gate2_delivery_p0 - step
   p1 - product
   silver_base_p1 blue_ring_p1 orange_ring_p1 black_cap_p1 gate2_delivery_p1 - step
   p2 - product
   blue_base_p2 gray_ring_p2 purple_ring_p2 yellow_ring_p2 black_ring_p2 green_ring_p2 red_ring_p2 brown_ring_p2 mint_ring_p2 gray_cap_p2 gate2_delivery_p2 - step
   )  
(:init
   (add-one zero one)	
   (add-one one two)
   (add-one two three)
   (subtract three zero three)
   (subtract two zero two)
   (subtract one zero one)
   (subtract zero zero zero)
   (subtract three one two)
   (subtract two one one)
   (subtract one one zero)
   (subtract three two one)
   (subtract two two zero)
   (has-step p0 silver_base_p0)
   (has-step p0 gray_cap_p0)
   (has-step p0 gate2_delivery_p0)
   (initial-step silver_base_p0)
   (has-step p1 silver_base_p1)
   (has-step p1 blue_ring_p1)
   (has-step p1 orange_ring_p1)
   (has-step p1 black_cap_p1)
   (has-step p1 gate2_delivery_p1)
   (initial-step silver_base_p1)
   (has-step p2 blue_base_p2)
   (has-step p2 gray_ring_p2)
   (has-step p2 purple_ring_p2)
   (has-step p2 yellow_ring_p2)
   (has-step p2 black_ring_p2)
   (has-step p2 green_ring_p2)
   (has-step p2 red_ring_p2)
   (has-step p2 brown_ring_p2)
   (has-step p2 mint_ring_p2)
   (has-step p2 gray_cap_p2)
   (has-step p2 gate2_delivery_p2)
   (initial-step blue_base_p2)
   (output-location bs0_out bs0)
   (input-location rs0_in rs0)
   (output-location rs0_out rs0)
   (input-location rs1_in rs1)
   (output-location rs1_out rs1)
   (input-location cs0_in cs0)
   (output-location cs0_out cs0)
   (input-location cs1_in cs1)
   (output-location cs1_out cs1)
   (input-location ds0_in ds0)
   (robot1-at-init)
   (robot1-assigned-machine bs0)
   (robot1-assigned-machine rs0)
   (robot1-assigned-machine rs1)
   (robot1-assigned-machine cs0)
   (robot1-assigned-machine cs1)
   (robot1-assigned-machine ds0)
   (step-at-machine silver_base_p0 bs0)
   (step-at-machine gray_cap_p0 cs1)
   (step-at-machine gate2_delivery_p0 ds0)
   (step-at-machine silver_base_p1 bs0)
   (step-at-machine blue_ring_p1 rs1)
   (step-at-machine orange_ring_p1 rs1)
   (step-at-machine black_cap_p1 cs0)
   (step-at-machine gate2_delivery_p1 ds0)
   (step-at-machine blue_base_p2 bs0)
   (step-at-machine gray_ring_p2 rs1)
   (step-at-machine purple_ring_p2 rs0)
   (step-at-machine yellow_ring_p2 rs0)
   (step-at-machine black_ring_p2 rs0)
   (step-at-machine green_ring_p2 rs1)
   (step-at-machine red_ring_p2 rs1)
   (step-at-machine brown_ring_p2 rs1)
   (step-at-machine mint_ring_p2 rs0)
   (step-at-machine gray_cap_p2 cs0)
   (step-at-machine gate2_delivery_p2 ds0)
   (step-precedes silver_base_p0 gray_cap_p0)
   (step-precedes gray_cap_p0 gate2_delivery_p0)
   (step-precedes silver_base_p1 blue_ring_p1)
   (step-precedes blue_ring_p1 orange_ring_p1)
   (step-precedes orange_ring_p1 black_cap_p1)
   (step-precedes black_cap_p1 gate2_delivery_p1)
   (step-precedes blue_base_p2 gray_ring_p2)
   (step-precedes gray_ring_p2 purple_ring_p2)
   (step-precedes purple_ring_p2 yellow_ring_p2)
   (step-precedes yellow_ring_p2 black_ring_p2)
   (step-precedes black_ring_p2 green_ring_p2)
   (step-precedes green_ring_p2 red_ring_p2)
   (step-precedes red_ring_p2 brown_ring_p2)
   (step-precedes brown_ring_p2 mint_ring_p2)
   (step-precedes mint_ring_p2 gray_cap_p2)
   (step-precedes gray_cap_p2 gate2_delivery_p2)
   (material-required silver_base_p0 three)
   (material-required silver_base_p1 zero)
   (material-required blue_base_p2 zero)
   (material-required purple_ring_p2 zero)
   (material-required yellow_ring_p2 one)
   (material-required black_ring_p2 zero)
   (material-required mint_ring_p2 zero)
   (material-required blue_ring_p1 two)
   (material-required orange_ring_p1 one)
   (material-required gray_ring_p2 zero)
   (material-required green_ring_p2 zero)
   (material-required red_ring_p2 zero)
   (material-required brown_ring_p2 zero)
   (material-required purple_ring_p2 two)
   (material-required yellow_ring_p2 zero)
   (material-required black_ring_p2 zero)
   (material-required mint_ring_p2 one)
   (material-required blue_ring_p1 zero)
   (material-required orange_ring_p1 zero)
   (material-required gray_ring_p2 zero)
   (material-required green_ring_p2 three)
   (material-required red_ring_p2 zero)
   (material-required brown_ring_p2 zero)
   (material-required silver_base_p0 zero)
   (material-required silver_base_p1 three)
   (material-required blue_base_p2 zero)
   (material-stored bs0 zero)
   (material-stored rs0 zero)
   (material-stored rs1 zero)
   (material-stored cs0 zero)
   (material-stored cs1 zero)
   (material-stored ds0 zero)
   (= (path-length start bs0_in) 14)
   (= (path-length bs0_in start) 14)
   (= (path-length start bs0_out) 14)
   (= (path-length bs0_out start) 14)
   (= (path-length start rs0_in) 16)
   (= (path-length rs0_in start) 16)
   (= (path-length start rs0_out) 15)
   (= (path-length rs0_out start) 15)
   (= (path-length start rs1_in) 9)
   (= (path-length rs1_in start) 9)
   (= (path-length start rs1_out) 10)
   (= (path-length rs1_out start) 10)
   (= (path-length start cs0_in) 28)
   (= (path-length cs0_in start) 28)
   (= (path-length start cs0_out) 27)
   (= (path-length cs0_out start) 27)
   (= (path-length start cs1_in) 20)
   (= (path-length cs1_in start) 20)
   (= (path-length start cs1_out) 21)
   (= (path-length cs1_out start) 21)
   (= (path-length start ds0_in) 25)
   (= (path-length ds0_in start) 25)
   (= (path-length bs0_in bs0_in) 0)
   (= (path-length bs0_in bs0_out) 2)
   (= (path-length bs0_out bs0_in) 2)
   (= (path-length bs0_out bs0_out) 0)
   (= (path-length bs0_in rs0_in) 4)
   (= (path-length bs0_in rs0_out) 2)
   (= (path-length bs0_out rs0_in) 5)
   (= (path-length bs0_out rs0_out) 4)
   (= (path-length bs0_in rs1_in) 9)
   (= (path-length bs0_in rs1_out) 9)
   (= (path-length bs0_out rs1_in) 8)
   (= (path-length bs0_out rs1_out) 9)
   (= (path-length bs0_in cs0_in) 19)
   (= (path-length bs0_in cs0_out) 17)
   (= (path-length bs0_out cs0_in) 20)
   (= (path-length bs0_out cs0_out) 19)
   (= (path-length bs0_in cs1_in) 12)
   (= (path-length bs0_in cs1_out) 13)
   (= (path-length bs0_out cs1_in) 10)
   (= (path-length bs0_out cs1_out) 12)
   (= (path-length bs0_in ds0_in) 12)
   (= (path-length bs0_out ds0_in) 11)
   (= (path-length rs0_in bs0_in) 4)
   (= (path-length rs0_in bs0_out) 5)
   (= (path-length rs0_out bs0_in) 2)
   (= (path-length rs0_out bs0_out) 4)
   (= (path-length rs0_in rs0_in) 0)
   (= (path-length rs0_in rs0_out) 2)
   (= (path-length rs0_out rs0_in) 2)
   (= (path-length rs0_out rs0_out) 0)
   (= (path-length rs0_in rs1_in) 11)
   (= (path-length rs0_in rs1_out) 13)
   (= (path-length rs0_out rs1_in) 11)
   (= (path-length rs0_out rs1_out) 11)
   (= (path-length rs0_in cs0_in) 15)
   (= (path-length rs0_in cs0_out) 14)
   (= (path-length rs0_out cs0_in) 17)
   (= (path-length rs0_out cs0_out) 15)
   (= (path-length rs0_in cs1_in) 14)
   (= (path-length rs0_in cs1_out) 15)
   (= (path-length rs0_out cs1_in) 13)
   (= (path-length rs0_out cs1_out) 14)
   (= (path-length rs0_in ds0_in) 12)
   (= (path-length rs0_out ds0_in) 11)
   (= (path-length rs1_in bs0_in) 9)
   (= (path-length rs1_in bs0_out) 8)
   (= (path-length rs1_out bs0_in) 9)
   (= (path-length rs1_out bs0_out) 9)
   (= (path-length rs1_in rs0_in) 11)
   (= (path-length rs1_in rs0_out) 11)
   (= (path-length rs1_out rs0_in) 13)
   (= (path-length rs1_out rs0_out) 11)
   (= (path-length rs1_in rs1_in) 0)
   (= (path-length rs1_in rs1_out) 2)
   (= (path-length rs1_out rs1_in) 2)
   (= (path-length rs1_out rs1_out) 0)
   (= (path-length rs1_in cs0_in) 26)
   (= (path-length rs1_in cs0_out) 25)
   (= (path-length rs1_out cs0_in) 27)
   (= (path-length rs1_out cs0_out) 26)
   (= (path-length rs1_in cs1_in) 12)
   (= (path-length rs1_in cs1_out) 13)
   (= (path-length rs1_out cs1_in) 11)
   (= (path-length rs1_out cs1_out) 12)
   (= (path-length rs1_in ds0_in) 18)
   (= (path-length rs1_out ds0_in) 18)
   (= (path-length cs0_in bs0_in) 19)
   (= (path-length cs0_in bs0_out) 20)
   (= (path-length cs0_out bs0_in) 17)
   (= (path-length cs0_out bs0_out) 19)
   (= (path-length cs0_in rs0_in) 15)
   (= (path-length cs0_in rs0_out) 17)
   (= (path-length cs0_out rs0_in) 14)
   (= (path-length cs0_out rs0_out) 15)
   (= (path-length cs0_in rs1_in) 26)
   (= (path-length cs0_in rs1_out) 27)
   (= (path-length cs0_out rs1_in) 25)
   (= (path-length cs0_out rs1_out) 26)
   (= (path-length cs0_in cs0_in) 0)
   (= (path-length cs0_in cs0_out) 2)
   (= (path-length cs0_out cs0_in) 2)
   (= (path-length cs0_out cs0_out) 0)
   (= (path-length cs0_in cs1_in) 28)
   (= (path-length cs0_in cs1_out) 29)
   (= (path-length cs0_out cs1_in) 26)
   (= (path-length cs0_out cs1_out) 28)
   (= (path-length cs0_in ds0_in) 20)
   (= (path-length cs0_out ds0_in) 19)
   (= (path-length cs1_in bs0_in) 12)
   (= (path-length cs1_in bs0_out) 10)
   (= (path-length cs1_out bs0_in) 13)
   (= (path-length cs1_out bs0_out) 12)
   (= (path-length cs1_in rs0_in) 14)
   (= (path-length cs1_in rs0_out) 13)
   (= (path-length cs1_out rs0_in) 15)
   (= (path-length cs1_out rs0_out) 14)
   (= (path-length cs1_in rs1_in) 12)
   (= (path-length cs1_in rs1_out) 11)
   (= (path-length cs1_out rs1_in) 13)
   (= (path-length cs1_out rs1_out) 12)
   (= (path-length cs1_in cs0_in) 28)
   (= (path-length cs1_in cs0_out) 26)
   (= (path-length cs1_out cs0_in) 29)
   (= (path-length cs1_out cs0_out) 28)
   (= (path-length cs1_in cs1_in) 0)
   (= (path-length cs1_in cs1_out) 2)
   (= (path-length cs1_out cs1_in) 2)
   (= (path-length cs1_out cs1_out) 0)
   (= (path-length cs1_in ds0_in) 10)
   (= (path-length cs1_out ds0_in) 12)
   (= (path-length ds0_in bs0_in) 12)
   (= (path-length ds0_in bs0_out) 11)
   (= (path-length ds0_in rs0_in) 12)
   (= (path-length ds0_in rs0_out) 11)
   (= (path-length ds0_in rs1_in) 18)
   (= (path-length ds0_in rs1_out) 18)
   (= (path-length ds0_in cs0_in) 20)
   (= (path-length ds0_in cs0_out) 19)
   (= (path-length ds0_in cs1_in) 10)
   (= (path-length ds0_in cs1_out) 12)
   (= (path-length ds0_in ds0_in) 0)
   (= (path-length start start) 0) 
   (= (total-cost) 0)
)
(:goal (and
   (step-completed silver_base_p0)
   (step-completed silver_base_p1)
   (step-completed blue_base_p2)
   (step-completed purple_ring_p2)
   (step-completed yellow_ring_p2)
   (step-completed black_ring_p2)
   (step-completed mint_ring_p2)
   (step-completed blue_ring_p1)
   (step-completed orange_ring_p1)
   (step-completed gray_ring_p2)
   (step-completed green_ring_p2)
   (step-completed red_ring_p2)
   (step-completed brown_ring_p2)
   (step-completed black_cap_p1)
   (step-completed gray_cap_p2)
   (step-completed gray_cap_p0)
   (step-completed gate2_delivery_p0)
   (step-completed gate2_delivery_p1)
   (step-completed gate2_delivery_p2)
   ))
(:metric minimize (total-cost))
)
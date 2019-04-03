(define (problem rcll-production-task)
(:domain rcll-production-steps)
(:objects
   r1 r2 r3 r4 r5 - robot
   p0 - product
   silver_base_p0 gray_cap_p0 gate2_delivery_p0 - step
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
   (robot2-at-init)
   (robot1-precedes r2)
   (robot1-precedes r3)
   (robot1-precedes r4)
   (robot1-precedes r5)
   (robot2-precedes r3)
   (robot2-precedes r4)
   (robot2-precedes r5)
   (robot3-precedes r4)
   (robot3-precedes r5)
   (robot4-precedes r5)
   (robot2-assigned-machine bs0)
   (robot2-assigned-machine rs1)
   (robot2-assigned-machine cs1)
   (step-at-machine silver_base_p0 bs0)
   (step-at-machine gray_cap_p0 cs0)
   (step-at-machine gate2_delivery_p0 ds0)
   (step-precedes silver_base_p0 gray_cap_p0)
   (step-precedes gray_cap_p0 gate2_delivery_p0)
   (material-required silver_base_p0 two)
   (material-required silver_base_p0 one)
   (material-stored bs0 zero)
   (material-stored rs0 zero)
   (material-stored rs1 zero)
   (material-stored cs0 zero)
   (material-stored cs1 zero)
   (material-stored ds0 zero)
   (= (path-length start bs0_in) 23)
   (= (path-length bs0_in start) 23)
   (= (path-length start bs0_out) 23)
   (= (path-length bs0_out start) 23)
   (= (path-length start rs0_in) 39)
   (= (path-length rs0_in start) 39)
   (= (path-length start rs0_out) 39)
   (= (path-length rs0_out start) 39)
   (= (path-length start rs1_in) 29)
   (= (path-length rs1_in start) 29)
   (= (path-length start rs1_out) 30)
   (= (path-length rs1_out start) 30)
   (= (path-length start cs0_in) 15)
   (= (path-length cs0_in start) 15)
   (= (path-length start cs0_out) 16)
   (= (path-length cs0_out start) 16)
   (= (path-length start cs1_in) 25)
   (= (path-length cs1_in start) 25)
   (= (path-length start cs1_out) 25)
   (= (path-length cs1_out start) 25)
   (= (path-length start ds0_in) 18)
   (= (path-length ds0_in start) 18)
   (= (path-length bs0_in bs0_in) 0)
   (= (path-length bs0_in bs0_out) 2)
   (= (path-length bs0_out bs0_in) 2)
   (= (path-length bs0_out bs0_out) 0)
   (= (path-length bs0_in rs0_in) 19)
   (= (path-length bs0_in rs0_out) 19)
   (= (path-length bs0_out rs0_in) 20)
   (= (path-length bs0_out rs0_out) 19)
   (= (path-length bs0_in rs1_in) 11)
   (= (path-length bs0_in rs1_out) 13)
   (= (path-length bs0_out rs1_in) 10)
   (= (path-length bs0_out rs1_out) 11)
   (= (path-length bs0_in cs0_in) 8)
   (= (path-length bs0_in cs0_out) 8)
   (= (path-length bs0_out cs0_in) 9)
   (= (path-length bs0_out cs0_out) 8)
   (= (path-length bs0_in cs1_in) 14)
   (= (path-length bs0_in cs1_out) 12)
   (= (path-length bs0_out cs1_in) 15)
   (= (path-length bs0_out cs1_out) 14)
   (= (path-length bs0_in ds0_in) 5)
   (= (path-length bs0_out ds0_in) 6)
   (= (path-length rs0_in bs0_in) 19)
   (= (path-length rs0_in bs0_out) 20)
   (= (path-length rs0_out bs0_in) 19)
   (= (path-length rs0_out bs0_out) 19)
   (= (path-length rs0_in rs0_in) 0)
   (= (path-length rs0_in rs0_out) 2)
   (= (path-length rs0_out rs0_in) 2)
   (= (path-length rs0_out rs0_out) 0)
   (= (path-length rs0_in rs1_in) 26)
   (= (path-length rs0_in rs1_out) 27)
   (= (path-length rs0_out rs1_in) 25)
   (= (path-length rs0_out rs1_out) 26)
   (= (path-length rs0_in cs0_in) 25)
   (= (path-length rs0_in cs0_out) 26)
   (= (path-length rs0_out cs0_in) 25)
   (= (path-length rs0_out cs0_out) 25)
   (= (path-length rs0_in cs1_in) 15)
   (= (path-length rs0_in cs1_out) 15)
   (= (path-length rs0_out cs1_in) 16)
   (= (path-length rs0_out cs1_out) 15)
   (= (path-length rs0_in ds0_in) 24)
   (= (path-length rs0_out ds0_in) 23)
   (= (path-length rs1_in bs0_in) 11)
   (= (path-length rs1_in bs0_out) 10)
   (= (path-length rs1_out bs0_in) 13)
   (= (path-length rs1_out bs0_out) 11)
   (= (path-length rs1_in rs0_in) 26)
   (= (path-length rs1_in rs0_out) 25)
   (= (path-length rs1_out rs0_in) 27)
   (= (path-length rs1_out rs0_out) 26)
   (= (path-length rs1_in rs1_in) 0)
   (= (path-length rs1_in rs1_out) 2)
   (= (path-length rs1_out rs1_in) 2)
   (= (path-length rs1_out rs1_out) 0)
   (= (path-length rs1_in cs0_in) 16)
   (= (path-length rs1_in cs0_out) 15)
   (= (path-length rs1_out cs0_in) 17)
   (= (path-length rs1_out cs0_out) 16)
   (= (path-length rs1_in cs1_in) 24)
   (= (path-length rs1_in cs1_out) 23)
   (= (path-length rs1_out cs1_in) 25)
   (= (path-length rs1_out cs1_out) 24)
   (= (path-length rs1_in ds0_in) 13)
   (= (path-length rs1_out ds0_in) 14)
   (= (path-length cs0_in bs0_in) 8)
   (= (path-length cs0_in bs0_out) 9)
   (= (path-length cs0_out bs0_in) 8)
   (= (path-length cs0_out bs0_out) 8)
   (= (path-length cs0_in rs0_in) 25)
   (= (path-length cs0_in rs0_out) 25)
   (= (path-length cs0_out rs0_in) 26)
   (= (path-length cs0_out rs0_out) 25)
   (= (path-length cs0_in rs1_in) 16)
   (= (path-length cs0_in rs1_out) 17)
   (= (path-length cs0_out rs1_in) 15)
   (= (path-length cs0_out rs1_out) 16)
   (= (path-length cs0_in cs0_in) 0)
   (= (path-length cs0_in cs0_out) 2)
   (= (path-length cs0_out cs0_in) 2)
   (= (path-length cs0_out cs0_out) 0)
   (= (path-length cs0_in cs1_in) 14)
   (= (path-length cs0_in cs1_out) 14)
   (= (path-length cs0_out cs1_in) 16)
   (= (path-length cs0_out cs1_out) 14)
   (= (path-length cs0_in ds0_in) 3)
   (= (path-length cs0_out ds0_in) 3)
   (= (path-length cs1_in bs0_in) 14)
   (= (path-length cs1_in bs0_out) 15)
   (= (path-length cs1_out bs0_in) 12)
   (= (path-length cs1_out bs0_out) 14)
   (= (path-length cs1_in rs0_in) 15)
   (= (path-length cs1_in rs0_out) 16)
   (= (path-length cs1_out rs0_in) 15)
   (= (path-length cs1_out rs0_out) 15)
   (= (path-length cs1_in rs1_in) 24)
   (= (path-length cs1_in rs1_out) 25)
   (= (path-length cs1_out rs1_in) 23)
   (= (path-length cs1_out rs1_out) 24)
   (= (path-length cs1_in cs0_in) 14)
   (= (path-length cs1_in cs0_out) 16)
   (= (path-length cs1_out cs0_in) 14)
   (= (path-length cs1_out cs0_out) 14)
   (= (path-length cs1_in cs1_in) 0)
   (= (path-length cs1_in cs1_out) 2)
   (= (path-length cs1_out cs1_in) 2)
   (= (path-length cs1_out cs1_out) 0)
   (= (path-length cs1_in ds0_in) 15)
   (= (path-length cs1_out ds0_in) 14)
   (= (path-length ds0_in bs0_in) 5)
   (= (path-length ds0_in bs0_out) 6)
   (= (path-length ds0_in rs0_in) 24)
   (= (path-length ds0_in rs0_out) 23)
   (= (path-length ds0_in rs1_in) 13)
   (= (path-length ds0_in rs1_out) 14)
   (= (path-length ds0_in cs0_in) 3)
   (= (path-length ds0_in cs0_out) 3)
   (= (path-length ds0_in cs1_in) 15)
   (= (path-length ds0_in cs1_out) 14)
   (= (path-length ds0_in ds0_in) 0)
   (= (path-length start start) 0) 
   (= (total-cost) 0)
)
(:goal (and
   (step-completed silver_base_p0)
   ))
(:metric minimize (total-cost))
)
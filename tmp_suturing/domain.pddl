(define (domain suture)
    (:requirements :strips :equality :negative-preconditions )
    (:predicates
		;  static predicates that do not change over time 
		; (Conf ?q)
		(IsArm ?arm)
		; (IsNeedle ?needle)
		; (Kin ?q ?p)
		(IsPose ?pose)
	    (AtPose ?arm ?pose)
		; (IsInsertPoint ?inPoint)
		; (isExtractPoint ?extPoint)

		; (isHolding ?arm-a ?needle-n)
		; (is-hand-empty ?arm-a )
		
		; (isExtracted ?needle ?extPoint)
		; (isPose ?pose)
		; (AtConf ?q)
		; (CanIK ?arm ?pose)
		; (CFree ?pose1 ?pose2 ?control)
		(HandEmpty ?arm)
		(isHolding ?arm ?needle)
		(Motion  ?arm ?pose2 ?control)
		(HoldMotion ?arm ?grasp ?loc ?control)
		(GraspMotion  ?arm ?pose ?control)
		(InsertMotion ?arm ?grasp ?loc ?control)
		(ExtractMotion ?arm ?grasp ?loc ?control)
		(IKvalid ?arm ?location ?grasp )
		(IsLocation ?loc)
		(CanReachLocation ?arm ?needle ?grasp)
		
		(IsNeedle ?needle)
		(IsTraj ?t)
		(IsGrasp ?grasp)
		(Unsafe ?control)
		(IsControl ?control)
		(isSutureMotion ?motion)
		(AtInsertion ?needle ?suturePoint)
		; (AtExtraction ?needle ?suturePoint)
		(isInserted ?needle ?location)
		(isSutured ?needle ?location)
		(AtLocation ?needle ?loc)
		(CanInsert ?arm ?grasp ?loc)
		(CanExtract ?arm ?grasp ?loc)
		; (hasGrasp ?arm ?grasp)
		; (IsSuturePoint ?p)

    	; Static predicates (predicates that do not change over time)








    )

	(:functions
    	(Distance ?arm ?pose)
  	)

    (:action move
    	:parameters (?arm ?pose ?pose2 ?control)
    	:precondition
    		(and (Motion ?arm ?pose2 ?control)
    			 ; (CanMove ?gripper) ; (Empty ?gripper)
    			 (AtPose ?arm ?pose) ;(not (Unsafe ?control))
				 )
    	:effect
    		(and (AtPose ?arm ?pose2)
    			 (not (AtPose ?arm ?pose))
    			;  (not (CanMove ?gripper))
    			 (increase (total-cost) (Distance ?control)))
    )

    (:action move_holding
    	:parameters (?arm ?needle ?loc1 ?loc2 ?grasp ?control)
    	:precondition
    		(and 
				(IKvalid ?arm ?loc1 ?grasp)
				(AtLocation ?needle ?loc1)
				(isHolding ?arm ?needle)
				; (CanReachLocation ?arm ?grasp ?loc2)
    			(HoldMotion ?arm ?grasp ?loc2 ?control)
				)
    	:effect
    		(and (AtLocation ?needle ?loc2)
    			 (not (AtLocation ?needle ?loc1))
    			 (increase (total-cost) (Distance ?control)))
    )
    
	(:action grasp_n
    	:parameters (?arm ?needle ?loc ?grasp ?control  )
    	:precondition
    		(and	
				(HandEmpty ?arm)
				(AtLocation ?needle ?loc)
				(IKvalid ?arm ?loc ?grasp)
				(GraspMotion ?arm ?grasp ?control)
    			
				; (not (isHolding ?arm ?needle))
				
				)
    	:effect
    		(and 
				(isHolding ?arm ?needle)
				(not (HandEmpty ?arm))
				(increase (total-cost) (Distance ?control))
				)
				
    )

	; assume that pose here is approach pose for the arm2
	; why do we need a grasp? 
	; (:action handover
    ; 	:parameters (?arm1 ?arm2 ?needle  )
    ; 	:precondition
    ; 		(and	
	; 			; (AtPose ?arm ?grasp)
	; 			; (IsArm ?arm)
	; 			; (IsPose ?grasp)
	; 			; (IK ?arm2 ?pose ?grasp)
	; 			; (GraspMotion ?arm2 ?grasp ?control)
	; 			; (Motion ?arm ?pose ?grasp ?control)
    ; 			(isHolding ?arm1 ?needle)
	; 			(HandEmpty ?arm2)
				
	; 			)
    ; 	:effect
    ; 		(and 
	; 			(isHolding ?arm2 ?needle)
	; 			(not (isHolding ?arm1 ?needle))
	; 			(not (HandEmpty ?arm2))
	; 			(HandEmpty ?arm1)
	; 			; (AtPose ?arm ?grasp)
    ; 			;  (not (AtPose ?arm ?pose))
	; 			(increase (total-cost) 0.1)
	; 			)
				
    ; )

	(:action insert_n
    	:parameters (?arm ?needle ?loc ?grasp ?control)
    	:precondition
    		(and 
				(isHolding ?arm ?needle)
				; (not (HandEmpty ?arm))
				(AtLocation ?needle ?loc)
				(CanInsert ?arm ?grasp ?loc)
    			(InsertMotion ?arm ?grasp ?loc ?control)
				
			)
    	:effect
    		(and 
				(isInserted ?needle ?loc)
				(not (isHolding ?arm ?needle))
				(HandEmpty ?arm)
				(increase (total-cost) (Distance ?control))
			)
				
    )
	
	(:action extract_n
    	:parameters (?arm ?needle ?loc ?grasp ?control)
    	:precondition
    		(and 
				
				(HandEmpty ?arm)
				(AtLocation ?needle ?loc)
				; (CanExtract ?arm ?grasp ?loc)
				; (not (isHolding ?arm ?needle))
				(isInserted ?needle ?loc)
    			(ExtractMotion ?arm ?grasp ?loc ?control)
			)
    	:effect
    		(and 
				(isSutured ?needle ?loc)
				; (isHolding ?arm ?needle)
				; (not (HandEmpty ?arm))
				(increase (total-cost) (Distance ?control))
			)
				
    )


    ; (:derived (Holding ?cup)
    ;     (exists (?grasp) (and (IsGrasp ?cup ?grasp)
    ;                           (Grasped ?cup ?grasp)))
    ; )
    ; (:derived (On ?cup ?block)
    ;     (exists (?pose ?pose2) (and (BlockSupport ?cup ?pose ?block ?pose2)
    ;                                 (AtPose ?cup ?pose) (AtPose ?block ?pose2)))
    ; )
)


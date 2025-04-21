class_name HybridTestBench
extends Node3D


var shiny_steel = preload("res://materials/shiny_steel.tres")
var base_colour = null


func _ready() -> void:
	base_colour = shiny_steel.albedo_color

func move_actuator():
	pass

func change_actuator_colour():
		shiny_steel.albedo_color = Color.LIGHT_CORAL

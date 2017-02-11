$(function() {
    /* setup */
    window_height= $(window).height();
    window_width = $(window).width();

    // Make an instance of two and place it on the page.
    var elem = document.getElementById('starlings');
    var params = { width: window_width, height: window_height };
    two = new Two(params).appendTo(elem);

    m = new Murmuration(50);

    /* Initial render */
    two.update();

    /********** Bindings **********/

    /* track mouse position for "predator" */
    $("#starlings").mousemove(function( event ) {
	m.predator.move( event.pageX , event.pageY );
    });

    /* general jquery bindings */
    $(window).bind('click',function() {
	for (var i = 0; i < 1; i ++) m.UpdateBirds();
	two.update();
    });

    /* two bindings */
    two.bind('update', function() {
	m.UpdateBirds();
    })
    .play();

});
